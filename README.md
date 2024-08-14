# Competitive-Programming-Cheat-Sheet

## 1. Sorting
- Sorting custom object which ```std::sort``` requires **<** operator to be defined by the class of that object, but keep in mind while defining **<** operator it follows weak comparison i.e if a < b and b < c then a < c.
 ```
 class Custom {
   public:
      bool operator<(const Custom& other) const {
          // do weak comparison here and return whether this object is less than the other or not
      }
  }
 ```
- ```std::stable_sort``` can be used if the order of input iterator is to be maintained while sorting other wise custom comparator can be used to do so.
- In quick sort, use two pointer to store all the elements greater than the pivot element while iterating over the array,
```
   // keep left pointer to the end of sequence less the pivot
   if (arr[i] < pivot) {
      left++;
      swap(arr[left], arr[i]);
   }
   right++;
```

- Interval Scheduling Maximization algorithm is used to select the maximum number of non-overlapping intervals (or activities) from a given set of intervals.The idea is to always pick the interval that ends the earliest (and doesn't overlap with previously selected intervals). This strategy ensures that you leave the most room for future intervals to be included. 


## 2. Binary Search
- Binary search can be applied even if the input space is unsorted, as long as the search space allows us to determine in constant time whether the target is present in a specific range. By iteratively halving the search space based on this condition, binary search becomes a feasible and efficient approach.
  
- If the result of an optimization strategy is linear, meaning that finding a higher optimal solution ensures the existence of all lower optimal solutions, then binary search can be applied over the entire range of optimal solutions—*provided that determining whether a given solution is optimal can be done in linear or constant time*.
  
 ```
  auto left = min_possible_search, right = max_possible_search;
  while (left <= right) {
    auto mid = (left + right) / 2;
    // function possible return bool, whether mid can be optimal solution or not
    if (possible(input_space.begin(), input_space.end(), mid) left = ++mid;
    else right = --mid;
  }
  return left - 1;
 ```

- Binary search can be used to find the last or the nth possible solution within the search space if it follows 1 1 1 1 1 1 0 0 0 0 0 0 0  pattern in serach space. (1 means possible and 0 means not possible)

- Any dynamic programming problem that involves finding optimal solution can be solved using binary search, it cannot be the optmized solution but it still be faster than the brute force. For example finding the longest common substring can be solved in O(n * m) * log (m) time which is slower than O(n * m).

- ```std::lower_bound(input_space.begin(), input_space.end(), target)``` returns the iterator pointing to the first element which is greater than or equal to the target if the input space is sorted by defining < operator while ```std::upper_bound(input_space.begin(), input_space.end(), target)``` returns the first element greater than the target. These two functions also take the optional custom lambda fucntion for comparision.

- ```std::set```, ```std::map``` and ```std::multiset``` also has ```lower_bound``` and ```upper_bound``` methods defined that can be used when the input space is mutable and changing over time.

- To find lower_bound and upper_bound of an element in the given input space, use this
```
... // a sorted array is stored as a[0], a[1], ..., a[n-1]
int lb = -1, ub = n;
while (ub - lb > 1) {
    int m = (lb + ub) / 2;
    if (k < a[m]) {
        ub = m; // a[l] <= k < a[m] <= a[r]
    } else {
        lb = m; // a[l] <= a[m] <= k < a[r]
    }
}
return {lb, ub};
```

## 3. String
- Computing hash of string can be used to determine whether two string are equal or not in O(n) time with the probability that collision happens is only $\approx \frac{1}{m}$ . For $m = 10^9 + 9$  the probability is $\approx 10^{-9}$  which is quite low.
```
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
```
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
