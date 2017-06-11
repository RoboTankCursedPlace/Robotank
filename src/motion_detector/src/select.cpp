#include "select.h"


int Partition(uint8_t* array, int left, int right, int pivotIndex)
{
	uint8_t pivotValue = array[pivotIndex];
	
	
	//  move pivot to end
	uint8_t temp = array[right];
	array[right] = pivotValue;
	array[pivotIndex] = temp;
	
	
	int r = left;
	for (int i = left; i < right; ++i)
	{
		if (array[i] < pivotValue)
		{
			temp = array[r];
			array[r] = array[i];
			array[i] = temp;
			++r;
		}
	}
	
	//  move plot to middle(ordered middle)
	temp = array[right];
	array[right] = array[r];
	array[r] = temp;
	return r;
}
	

uint8_t QuickSelectRecursive(uint8_t* array, int left, int right, int k)
{
    if (left == right)
        return array[left];
        
    //  Select middle element as pivot
    //Maybe randomize one is better?
    int pivotIndex = (left+right) / 2;

    pivotIndex = Partition(array, left, right, pivotIndex);
    
    if (k == pivotIndex)
    {
		return array[k];
	}
	else if (k < pivotIndex)
	{
		return QuickSelectRecursive(array, left, pivotIndex-1, k);
	}
	else
	{
		return QuickSelectRecursive(array, pivotIndex+1, right, k);
	}
}


//  Iterative version is better, but not implemented yet!
uint8_t QuickSelectIterative(uint8_t* array, int left, int right, int k)
{
	return 0;
}
