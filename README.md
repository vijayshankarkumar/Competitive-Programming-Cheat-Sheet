# Competitive Programming Cheat Sheet

## 0. Invariant
- An invariant of an algorithm is a condition that remains true throughout the execution of the algorithm, typically within a loop or recursive call. Invariants are crucial in ensuring that an algorithm works correctly and terminates as expected.

- When designing any algorithm first define the invariants and check after every step in the execution flow, does invariants hold true, if it does then that algorithm is correct, robust and must stop in finite steps.
```cpp
// For example lets see the invariant of binary search, find whether an element x is present in the
// sorted array of integers a[0...n - 1]
int l = 0, r = n - 1;
// Invariant: if the given element is present in the array, it must be in between index l and r
//            i.e. each step in the execution flow a[l] < x && a[r] > x.
//            If the given element is present in the array then l == r hold true in the end.
while (l <= r) {
   int m = l + (r - l) / 2;
   if (a[m] == x) return true;
   if (a[m] < x) l = m + 1;  // Here invairant holds true as the array is sorted and
                             // if the target element is present,
                             // then it should be in the other half of the range [l, r]
   else r = m - 1;
}
return false;
```

- For example, in Dijkstra's algorithm, the main invariant is that once a node's shortest path estimate is marked as "final" (i.e., the node is added to the set of processed nodes), this estimate is indeed the shortest possible path from the source to that node. This invariant ensures the correctness of the algorithm. 

## 1. Sorting
- Sorting custom object which ```std::sort``` requires **<** operator to be defined by the class of that object, but keep in mind while defining **<** operator it follows weak comparison i.e if a < b and b < c then a < c.
 ```cpp
 class Custom {
   public:
      bool operator<(const Custom& other) const {
          // do weak comparison here and return whether
          // this object is less than the other or not
      }
  }
 ```
- ```std::stable_sort``` can be used if the order of input iterator is to be maintained while sorting other wise custom comparator can be used to do so.
- In quick sort, use two pointer to store all the elements greater than the pivot element while iterating over the array,
```cpp
   // keep left pointer to the end of sequence less the pivot and
   // right pointer to the start of the sequence which is greater or equal to pivot
   if (arr[right] < pivot) {
      left++;
      swap(arr[left], arr[right]);
   }
   right++;
```

- Interval Scheduling Maximization algorithm is used to select the maximum number of non-overlapping intervals (or activities) from a given set of intervals.The idea is to always pick the interval that ends the earliest (and doesn't overlap with previously selected intervals). This strategy ensures that you leave the most room for future intervals to be included. 


## 2. Binary Search
- Designing binary search solution can be tricky as one can make mistake in defining the invariants or somes step does not holds true for the defined invaraints during the execution  of the algorithm.
```cpp
// Find the index of the given element x in the sorted array a[0....n - 1] if it exists.
int l = 0, r = n - 1;
// Invariant: l -> points to the last element which is less than x.
              r -> points to the first element which is greater than x.
              In the end return if the target is present the l and r converge to a single index
while (l <= r) {
   int m = l + (r - l) / 2;
   if (a[m] == x) return m;
   else if (a[m] < x) l = m + 1; // Note that m is always in the range [0, n -1]
                                 // as m is calculated using integer division
   else r = m - 1;
}
return -1;
```
   
- Binary search can be applied even if the input space is unsorted, as long as the search space allows us to determine in constant time whether the target is present in a specific range. By iteratively halving the search space based on this condition, binary search becomes a feasible and efficient approach.
  
- If the result of an optimization strategy is linear, meaning that finding a higher optimal solution ensures the existence of all lower optimal solutions, then binary search can be applied over the entire range of optimal solutions—*provided that determining whether a given solution is optimal can be done in linear or constant time*.
  
 ```cpp
  auto left = min_possible_search, right = max_possible_search;
  // Invariant:  left -> points to last solution which is possible
  //              right -> points to first solution which is not possible
  while (right - left > 1) {
    auto mid = l + (r - l) / 2;
    // function possible return bool, whether mid can be optimal solution or not
    if (possible(input_space.begin(), input_space.end(), mid) left = mid;
    else right = mid;
  }
  return left;
 ```

- Binary search can be used to find the last or the nth possible solution within the search space if it follows 1 1 1 1 1 1 0 0 0 0 0 0 0  pattern in serach space. (1 means possible and 0 means not possible)

- Any dynamic programming problem that involves finding optimal solution can be solved using binary search, it cannot be the optmized solution but it still be faster than the brute force. For example finding the longest common substring can be solved in O(n * m) * log (m) time which is slower than O(n * m).

- ```std::lower_bound(input_space.begin(), input_space.end(), target)``` returns the iterator pointing to the first element which is greater than or equal to the target if the input space is sorted by defining < operator while ```std::upper_bound(input_space.begin(), input_space.end(), target)``` returns the first element greater than the target. These two functions also take the optional custom lambda fucntion for comparision.

- ```std::set```, ```std::map``` and ```std::multiset``` also has ```lower_bound``` and ```upper_bound``` methods defined that can be used when the input space is mutable and changing over time.

```cpp
// This is the stl implementation of std::lower_bound...
 _DistanceType __len = std::distance(__first, __last);
while (__len > 0)
{
	  _DistanceType __half = __len >> 1;
	  _ForwardIterator __middle = __first;
	  std::advance(__middle, __half);
	  if (__comp(__middle, __val))  // While defining custom comparator for std::lower_bound,
                                        // follow this comparator function
	    {                           // defnition. It should return true __middle < __val
	      __first = __middle;
	      ++__first;
	      __len = __len - __half - 1;
	    }
	  else
	    __len = __half;
}
return __first;

// Define comparator function like this
bool operator<(const Custom& obj, T val) const {
   // return true if obj is less than val
}
```

- To find lower_bound and upper_bound of an element in the given input space, use this
```cpp
... // a sorted array is stored as a[0], a[1], ..., a[n-1]
int lb = -1, ub = n;
// Invariant: lb -> points to the first element which is greater than or equal to the target x
//            ub -> points to the first element which is greater than the target x
while (ub - lb > 1) {
    int m = lb + (ub - lb) / 2;
    if (a[m] <= x) lb = m;
    else ub = m;
}
return {lb, ub};
```

## 3. String
- Computing hash of string can be used to determine whether two string are equal or not in O(n) time with the probability that collision happens is only $\approx \frac{1}{m}$ . For $m = 10^9 + 9$  the probability is $\approx 10^{-9}$  which is quite low.
```cpp
long long compute_hash(const string& s) {
    const int p = 31;
    const int m = 1e9 + 9;
    long long hash_value = 0;
    long long p_pow = 1;
    for (char c : s) {
        hash_value = (hash_value + (c - 'a' + 1) * p_pow) % m;
        p_pow = (p_pow * p) % m;
    }
    return hash_value;
}
```

- Finding longest prefix which is also the suffix at every index in the pattern, is used in KMP string matching algorithm, which is computed like this,
```cpp
 vector<int> prefix_function(const string& s) {
    vector<int> pi(s.size());
    for (int i = 1; i < s.size(); i++) {
        int j = pi[i-1];
        while (j > 0 && s[i] != s[j]) j = pi[j-1];
        if (s[i] == s[j]) j++;
        pi[i] = j;
    }
    return pi;
}
```
