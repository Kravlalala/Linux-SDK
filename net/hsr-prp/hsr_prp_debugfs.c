/*
 * hsr_prp_debugfs code
 * Copyright (C) 2017 Texas Instruments Incorporated
 *
 * Author(s):
 *	Murali Karicheri <m-karicheri2@ti.com?
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/debugfs.h>
#include "hsr_prp_main.h"
#include "hsr_prp_framereg.h"

static void print_mac_address(struct seq_file *sfp, unsigned char *mac)
{
	seq_printf(sfp, "%02x:%02x:%02x:%02x:%02x:%02x:",
		   mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/* hsr_prp_node_table_show - Formats and prints node_table entries
 */
static int
hsr_prp_node_table_show(struct seq_file *sfp, void *data)
{
	struct hsr_prp_priv *priv = (struct hsr_prp_priv *)sfp->private;
	struct hsr_prp_node *node;

	seq_puts(sfp, "Node Table entries\n");
	seq_puts(sfp, "MAC-Address-A,   MAC-Address-B, time_in[A], ");
	seq_puts(sfp, "time_in[B], Address-B port");
	if (priv->prot_version == PRP_V1)
		seq_puts(sfp, ", san_a, san_b\n");
	else
		seq_puts(sfp, "\n");
	rcu_read_lock();
	list_for_each_entry_rcu(node, &priv->node_db, mac_list) {
		/* skip self node */
		if (hsr_prp_addr_is_self(priv, node->mac_address_a))
			continue;
		print_mac_address(sfp, &node->mac_address_a[0]);
		seq_puts(sfp, " ");
		print_mac_address(sfp, &node->mac_address_b[0]);
		seq_printf(sfp, "0x%lx, ", node->time_in[HSR_PRP_PT_SLAVE_A]);
		seq_printf(sfp, "0x%lx ", node->time_in[HSR_PRP_PT_SLAVE_B]);
		seq_printf(sfp, "0x%x", node->addr_b_port);

		if (priv->prot_version == PRP_V1)
			seq_printf(sfp, ", %x, %x\n", node->san_a, node->san_b);
		else
			seq_puts(sfp, "\n");
	}
	rcu_read_unlock();
	return 0;
}

/* hsr_prp_node_table_open - Open the node_table file
 *
 * Description:
 * This routine opens a debugfs file node_table of specific hsr
 * or prp device
 */
static int
hsr_prp_node_table_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, hsr_prp_node_table_show, inode->i_private);
}

static const struct file_operations hsr_prp_node_table_fops = {
	.owner	= THIS_MODULE,
	.open	= hsr_prp_node_table_open,
	.read	= seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/* hsr_prp_stats_show - Formats and prints stats in the device
 */
static int
hsr_prp_stats_show(struct seq_file *sfp, void *data)
{
	struct hsr_prp_priv *priv = (struct hsr_prp_priv *)sfp->private;
	struct hsr_prp_port *master;

	rcu_read_lock();
	master = hsr_prp_get_port(priv, HSR_PRP_PT_MASTER);
	rcu_read_unlock();

	seq_puts(sfp, "Stats entries\n");
	seq_printf(sfp, "cnt_tx_a = %d\n", priv->stats.cnt_tx_a);
	seq_printf(sfp, "cnt_tx_b = %d\n", priv->stats.cnt_tx_b);
	seq_printf(sfp, "cnt_tx_c = %ld\n", master->dev->stats.tx_packets);
	seq_printf(sfp, "cnt_rx_wrong_lan_a = %d\n",
		   priv->stats.cnt_rx_wrong_lan_a);
	seq_printf(sfp, "cnt_rx_wrong_lan_b = %d\n",
		   priv->stats.cnt_rx_wrong_lan_b);
	seq_printf(sfp, "cnt_rx_a = %d\n", priv->stats.cnt_rx_a);
	seq_printf(sfp, "cnt_rx_b = %d\n", priv->stats.cnt_rx_b);
	seq_printf(sfp, "cnt_rx_c = %ld\n", master->dev->stats.rx_packets);
	seq_printf(sfp, "cnt_rx_errors_a = %d\n", priv->stats.cnt_rx_errors_a);
	seq_printf(sfp, "cnt_rx_errors_b = %d\n", priv->stats.cnt_rx_errors_b);
	if (priv->prot_version <= HSR_V1) {
		seq_printf(sfp, "cnt_own_rx_a = %d\n",
			   priv->stats.cnt_own_rx_a);
		seq_printf(sfp, "cnt_own_rx_b = %d\n",
			   priv->stats.cnt_own_rx_b);
	}
	seq_puts(sfp, "\n");
	return 0;
}

/* hsr_prp_stats_open - open stats file
 *
 * Description:
 * This routine opens a debugfs file stats of specific hsr or
 * prp device
 */
static int
hsr_prp_stats_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, hsr_prp_stats_show, inode->i_private);
}

static const struct file_operations hsr_prp_stats_fops = {
	.owner	= THIS_MODULE,
	.open	= hsr_prp_stats_open,
	.read	= seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
/* hsr_prp_debugfs_init - create hsr-prp node_table file for dumping
 * the node table
 *
 * Description:
 * When debugfs is configured this routine sets up the node_table file per
 * hsr/prp device for dumping the node_table entries
 */
int hsr_prp_debugfs_init(struct hsr_prp_priv *priv,
			 struct net_device *hsr_prp_dev)
{
	int rc = -1;
	struct dentry *de = NULL;

	if (priv->prot_version <= HSR_V1)
		de = debugfs_create_dir("hsr", NULL);
	else
		de = debugfs_create_dir("prp", NULL);

	if (!de) {
		netdev_err(hsr_prp_dev, "Cannot create hsr-prp debugfs root\n");
		return rc;
	}

	priv->root_dir = de;

	de = debugfs_create_file("node_table", S_IFREG | S_IRUGO,
				 priv->root_dir, priv,
				 &hsr_prp_node_table_fops);
	if (!de) {
		netdev_err(hsr_prp_dev,
			   "Cannot create hsr-prp node_table directory\n");
		return rc;
	}
	priv->node_tbl_file = de;

	de = debugfs_create_file("stats", S_IFREG | S_IRUGO,
				 priv->root_dir, priv,
				 &hsr_prp_stats_fops);
	if (!de) {
		netdev_err(hsr_prp_dev,
			   "Cannot create hsr-prp stats directory\n");
		return rc;
	}
	priv->stats_file = de;

	return 0;
} /* end of hst_prp_debugfs_init */

/* hsr_prp_debugfs_term - Tear down debugfs intrastructure
 *
 * Description:
 * When Debufs is configured this routine removes debugfs file system
 * elements that are specific to hsr-prp
 */
void
hsr_prp_debugfs_term(struct hsr_prp_priv *priv)
{
	debugfs_remove(priv->node_tbl_file);
	priv->node_tbl_file = NULL;
	debugfs_remove(priv->stats_file);
	priv->stats_file = NULL;
	debugfs_remove(priv->root_dir);
	priv->root_dir = NULL;
}
