#ifndef SELECT_H
#define SELECT_H

#include <stdint.h>


//  Recursive quick select algorithm
//Taken from wikipedia
//https://en.wikipedia.org/wiki/Quickselect
uint8_t QuickSelectRecursive(uint8_t* array, int left, int right, int k);


//  Iterative version is better, but not implemented yet!
uint8_t QuickSelectIterative(uint8_t* array, int left, int right, int k);



#endif
