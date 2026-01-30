import pandas as pd
from glob import glob
from tqdm import tqdm

res = pd.DataFrame()

for file in tqdm(glob('bag/*/*/*.csv')):
    print(file)
    res_ = pd.DataFrame()
    df = pd.read_csv(file)
    for key1 in df.keys():
        key2 = df[key1][0]
        key3 = df[key1][1]
        key = key1.split('.')[0]
        if key2 == '__msgtype__':
            continue
        if 'desired' in key and 'pressure' in key2:
            print(key+'/'+key2)
            l = []
            for i in range(2,len(df[key1])):
                l_ = df[key1][i].replace('[','').replace(']','').replace(',',' ').split()
                l_ = l_[:-2]
                l_ = list(map(float, l_))
                l.append(l_)
            df_ = pd.DataFrame(l, columns=[key+'/'+key2+str(i) for i in range(len(l[0]))])
            res_ = pd.concat([res_,df_],axis=1)
        if 'rigidbody1' in key and 'pos' in key2:
            print(key+'/'+key2)
            l = []
            for i in range(2,len(df[key1])):
                l_ = df[key1][i].replace('[','').replace(']','').replace(',',' ').split()
                l_ = list(map(float, l_))
                l.append(l_)
            df_ = pd.DataFrame(l, columns=[key+'/'+key2+str(i) for i in range(len(l[0]))])
            res_ =  pd.concat([res_,df_],axis=1)
    print(res_.shape)
    res = pd.concat([res,res_],axis=0,ignore_index=True)
    print(res.shape)

res.to_csv('data.csv',index=False)
