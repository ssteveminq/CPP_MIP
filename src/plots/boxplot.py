import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd

nums=[1,2,3]
x1 = [111, 119, 120, 123, 121, 126, 128]
x2 = [39, 41, 45, 44, 49, 42.6, 42]
x3 = [29, 26.5, 32, 33, 31, 32, 28]

x_mean= np.mean(x1)
x2_mean= np.mean(x2)
x3_mean= np.mean(x3)
# x2 = 10*np.random.exponential(0.5, 100)
# x5 = 10*np.random.normal(0, 0.4, 100)
# x10 = 10*np.random.normal(0, 0.4, 100)
a=plt.boxplot ([x1, x2, x3] )
for whisker in a['whiskers']:
    whisker.set_linestyle('-.')
    whisker.set_linewidth(3.0)
    print(whisker.get_linestyle())
# df = pd.DataFrame(nums,columns='n')
# df2 = pd.DataFrame([x1,x2,x3],columns='s')
# ax=plt.axes
# sns.boxplot(x=df['n'], y=df2['s'], linewidth=3)
plt.plot([1,2,3],[x_mean, x2_mean, x3_mean])
plt.show()

