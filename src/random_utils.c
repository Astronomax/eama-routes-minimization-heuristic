#include "random_utils.h"
#include "utils.h"

void
random_subset(struct customer **arr, int n, int k)
{
	for (int i = 0; i < k; i++) {
		int j = (int)pseudo_random_in_range(i, n - 1);
		SWAP(arr[i], arr[j]);
	}
}

void
random_shuffle(struct customer **arr, int n)
{
	random_subset(arr, n, n - 1);
}
