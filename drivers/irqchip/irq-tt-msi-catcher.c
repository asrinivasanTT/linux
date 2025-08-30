// SPDX-License-Identifier: GPL-2.0
/*
 * Tenstorrent MSI Catcher Driver
 * 
 * This driver implements the MSI Catcher, a small memory-mapped FIFO
 * attached to the platform-level interrupt controller (PLIC).
 * The MSI catcher provides a convenient way to atomically raise an 
 * interrupt via the NoC.
 *
 * Copyright (C) 2025 Tenstorrent AI ULC
 */

#define pr_fmt(fmt) "tt-msi-catcher: " fmt

#include <linux/cpu.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/irqdesc.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/of_device.h>

/*
 * MSI Catcher register offsets
 */
#define MSI_CATCHER_FIFO_REG		0x0000	/* FIFO read/write register */
#define MSI_CATCHER_CLEAR_REG		0x0004	/* FIFO clear register (read only) */
#define MSI_CATCHER_STATUS_REG		0x0008	/* Status register */
#define MSI_CATCHER_HWM_REG		0x000C	/* High water mark register */

/*
 * Status register bit definitions
 */
#define MSI_CATCHER_STATUS_NOT_EMPTY	BIT(8)	/* queue.size() != 0 */

#define MSI_CATCHER_MAX_IRQS		256

struct msi_catcher_priv {
	struct device *dev;
	void __iomem *base;
	struct irq_domain *domain;
	unsigned int parent_irq_empty;	/* Parent IRQ when FIFO not empty (PLIC #6) */
	unsigned int parent_irq_hwm;	/* Parent IRQ when FIFO >= HWM (PLIC #7) */
	raw_spinlock_t lock;
	u32 hwm;
};

static struct msi_catcher_priv *msi_catcher_priv_data;

static inline u32 msi_catcher_read_status(struct msi_catcher_priv *priv)
{
	return readl(priv->base + MSI_CATCHER_STATUS_REG);
}

static inline void msi_catcher_write_hwm(struct msi_catcher_priv *priv, u32 hwm)
{
	writel(hwm, priv->base + MSI_CATCHER_HWM_REG);
	priv->hwm = hwm;
}

/*
 * Check if FIFO is empty
 */
static bool msi_catcher_is_empty(struct msi_catcher_priv *priv)
{
	u32 status = msi_catcher_read_status(priv);
	return !(status & MSI_CATCHER_STATUS_NOT_EMPTY);
}

/*
 * IRQ chip operations for MSI catcher
 */
static void msi_catcher_irq_mask(struct irq_data *data)
{
	/* Individual MSI interrupts cannot be masked at the catcher level */
}

static void msi_catcher_irq_unmask(struct irq_data *data)
{
	/* Individual MSI interrupts cannot be unmasked at the catcher level */
}

static void msi_catcher_irq_ack(struct irq_data *data)
{
	/* ACK is handled by reading from FIFO in the interrupt handler */
}

static struct irq_chip msi_catcher_irq_chip = {
	.name		= "tt-msi-catcher",
	.irq_mask	= msi_catcher_irq_mask,
	.irq_unmask	= msi_catcher_irq_unmask,
	.irq_ack	= msi_catcher_irq_ack,
};

static int msi_catcher_domain_alloc(struct irq_domain *domain, unsigned int virq,
				 unsigned int nr_irqs, void *arg)
{
	int i, ret;
	irq_hw_number_t hwirq;
	unsigned int type;
	struct irq_fwspec *fwspec = arg;
    
	ret = irq_domain_translate_onecell(domain, fwspec, &hwirq, &type);
	if (ret)
    return ret;

    pr_err("Alloc hwirq: %d virq: %d", hwirq, virq);

	for (i = 0; i < nr_irqs; i++) {
        irq_domain_set_info(domain, virq + i, hwirq + i, &msi_catcher_irq_chip, domain->host_data,
			    handle_edge_irq, NULL, NULL);
	}

	return 0;
}


static const struct irq_domain_ops msi_catcher_domain_ops = {
	.translate	= irq_domain_translate_onecell,
	.alloc		= msi_catcher_domain_alloc,
	.free		= irq_domain_free_irqs_top,
};

/*
 * Main interrupt handler for MSI catcher
 * This handles both FIFO not empty (PLIC #6) and HWM reached (PLIC #7) interrupts
 */
static void msi_catcher_handle_irq(struct irq_desc *desc)
{
	struct msi_catcher_priv *priv = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	u32 msi_data;
	unsigned int virq;
	unsigned long flags;

	chained_irq_enter(chip, desc);

	raw_spin_lock_irqsave(&priv->lock, flags);

	/*
	 * Process all MSI messages in the FIFO
	 * Each read from the FIFO register pops one entry
	 */
	while (!msi_catcher_is_empty(priv)) {
		msi_data = readl(priv->base + MSI_CATCHER_FIFO_REG);

        /* Convert to actual virtual IRQ number in our domain */
		virq = irq_find_mapping(priv->domain, msi_data);

        // pr_err("msi_data: 0x%08x virq: %d", msi_data, virq);
		
		if (virq) {
            generic_handle_irq(virq);
		} else {
			dev_warn_ratelimited(priv->dev,
					      "Unmapped MSI data: 0x%08x (irq=%u)\n",
					      msi_data, msi_data);
		}
	}

	raw_spin_unlock_irqrestore(&priv->lock, flags);

	chained_irq_exit(chip, desc);
}

static int msi_catcher_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct msi_catcher_priv *priv;
	struct resource *res;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	raw_spin_lock_init(&priv->lock);

	/* Map MSI catcher registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	/* Get parent interrupts (PLIC sources #6 and #7) */
	priv->parent_irq_empty = irq_of_parse_and_map(node, 0);
	if (!priv->parent_irq_empty) {
		dev_err(dev, "Failed to get parent IRQ for FIFO not empty\n");
		return -EINVAL;
	}
	priv->parent_irq_hwm = irq_of_parse_and_map(node, 1);
	if (!priv->parent_irq_hwm) {
		dev_err(dev, "Failed to get parent IRQ for HWM\n");
		return -EINVAL;
	}

	/* Clear fifo */
	readl(priv->base + MSI_CATCHER_CLEAR_REG);

	/* Create IRQ domain */
	priv->domain = irq_domain_add_linear(node, MSI_CATCHER_MAX_IRQS,
					     &msi_catcher_domain_ops, priv);
	if (!priv->domain) {
		dev_err(dev, "Failed to create IRQ domain\n");
		return -ENOMEM;
	}

	/* Set up chained interrupt handlers */
	irq_set_chained_handler_and_data(priv->parent_irq_empty,
					 msi_catcher_handle_irq, priv);
	irq_set_chained_handler_and_data(priv->parent_irq_hwm,
					 msi_catcher_handle_irq, priv);

	platform_set_drvdata(pdev, priv);
	msi_catcher_priv_data = priv;

	dev_info(dev, "MSI Catcher initialized at %pR, parent IRQs %u,%u\n",
		 res, priv->parent_irq_empty, priv->parent_irq_hwm);

	return 0;
}

static const struct of_device_id msi_catcher_of_match[] = {
	{ .compatible = "tenstorrent,msi-catcher" },
	{}
};

static struct platform_driver msi_catcher_driver = {
    .driver	= {
        .name = "tt-msi-catcher",
		.of_match_table = msi_catcher_of_match,
	},
    .probe	= msi_catcher_probe,
};
builtin_platform_driver(msi_catcher_driver);

// static int msi_catcher_early_probe(struct device_node *node, struct device_node *parent)
// {
//     return msi_catcher_probe(to_platform_device((&node->fwnode)->dev));
// }

// /* Register with IRQCHIP framework for early initialization */
// IRQCHIP_DECLARE(tt_msi_catcher, "tenstorrent,msi-catcher", msi_catcher_early_probe);
