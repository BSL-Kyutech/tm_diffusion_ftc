import pandas as pd
from robotdiffusion import Diffuser


actuator_columns =[]
for i in range(4):
    for j in range(10):
        column = f'/tenpa/pressure/desired{i}/pressure{j}'
        actuator_columns.append(column)

actuator_max_abs_values = [255 for i in range(len(actuator_columns))]
main_target_columns = []
for i in range(3):
    column = f'/mocap/rigidbody1/pos{i}'
    main_target_columns.append(column)

#for i in range(4):
#    column = f'/mocap/rigidbody1/rot{i}'
#    main_target_columns.append(column)

model = Diffuser(actuator_columns, main_target_columns, actuator_max_abs_values)
train_df = pd.read_csv("train.csv")
test_df = pd.read_csv("test.csv")
model.train(train_df,test_df=test_df)
model.save_dir('model')