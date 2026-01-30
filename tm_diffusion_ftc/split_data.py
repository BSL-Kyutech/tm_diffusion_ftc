from sklearn.model_selection import train_test_split
import pandas as pd

df = pd.read_csv('data.csv')
train_df, test_df = train_test_split(df, test_size=0.2)
train_df.to_csv('train.csv',index=False)
test_df.to_csv('test.csv',index=False)