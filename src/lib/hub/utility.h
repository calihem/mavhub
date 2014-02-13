#ifndef _HUB_UTILITY_H_
#define _HUB_UTILITY_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif // HAVE_CONFIG_H


namespace hub {

/**
 * \brief Simple struct combining data item with an index.
 */
template<typename T>
struct indexed_item_t {
	indexed_item_t() {};
	indexed_item_t(const T &item, const uint16_t index) :
		item(item),
		index(index) {};
	friend bool operator<(const indexed_item_t &lhs, const indexed_item_t &rhs) {
		return lhs.item < rhs.item;
	}

	T item;
	uint16_t index;
};


} // namespace hub

#endif // _HUB_UTILITY_H_
