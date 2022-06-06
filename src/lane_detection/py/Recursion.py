# from random import shuffle

# lst = [1,0,0,0,0,0,0,0]
# c=0
# def find(lst):
#     global c
#     if len(lst) == 1:
#         return
#     m = len(lst)//2
#     if 1 in lst[:m]:
#         c +=1
#         return find(lst[:m])
#     c +=1    
#     return find(lst[m:])
# find(lst)
# print(c)
# print("")

# import random
# lst = [1,1,1,2,3,4,5,56,7,8,9,56]
# print(lst)
# print(max(lst))
# def find_max(A):
#     if len(A) >= 2:
#         a = A[0]
#         b = A[1]    
#         if a > b:
#             A[1] = a
#         else:
#             A[1] = b
#         return find_max(A[1:])
#     else:
#         return A[0]
# print(find_max(lst))

# def gen_subsets(k):

#     if k == n:
#         print(S)
#     else:
#         S.append(k)
#         gen_subsets(k+1)
#         S.pop()
#         gen_subsets(k+1)
# S = []
# n = int(input())
# gen_subsets(0)

# def power3(a,n):
#     if n == 0:
#         return 1
#     if n == 1:
#         return a
#     x = power3(a,n//2)
#     if n % 2 == 0:
#         return x*x
#     else:
#         return x*x*a # at lasttime multiply a
# print(power3(6,5))
# print(6*6*6*6)
lst = [2,3,9,10,17,28,31,45]
def bs(A,a,b,k):
    if a > b : return -1
    m = (a+b)//2
    if A[m] == k: return m
    elif A[m] > k : return bs(A,a,m-1,k)
    else: return bs(A,m+1,b,k)

print(bs(lst,0,7,41))