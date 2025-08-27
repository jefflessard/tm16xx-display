#ifndef _TM16XX_COMPAT_H
#define _TM16XX_COMPAT_H

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
static __always_inline
unsigned long bitmap_read(const unsigned long *map, unsigned long start, unsigned long nbits)
{
	size_t index = BIT_WORD(start);
	unsigned long offset = start % BITS_PER_LONG;
	unsigned long space = BITS_PER_LONG - offset;
	unsigned long value_low, value_high;

	if (unlikely(!nbits || nbits > BITS_PER_LONG))
		return 0;

	if (space >= nbits)
		return (map[index] >> offset) & BITMAP_LAST_WORD_MASK(nbits);

	value_low = map[index] & BITMAP_FIRST_WORD_MASK(start);
	value_high = map[index + 1] & BITMAP_LAST_WORD_MASK(start + nbits);
	return (value_low >> offset) | (value_high << space);
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 16, 0)
static unsigned int fwnode_get_child_node_count(const struct fwnode_handle *fwnode)
{
	struct fwnode_handle *child;
	unsigned int count = 0;

	fwnode_for_each_child_node(fwnode, child)
		count++;

	return count;
}
#endif

#endif /* _TM16XX_COMPAT_H */
