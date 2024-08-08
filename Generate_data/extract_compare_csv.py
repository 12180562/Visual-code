import pandas as pd



df1 = pd.read_csv("c:/Users/muyeong/Desktop/simulation_0730/generate_data.csv")

df1.columns = ['time', '.m_nShipID', '.m_fltHeading', '.m_fltVel_U', '.m_fltPos_X', '.m_fltPos_Y', '.RD','.RC', '.K', '.DCPA', '.TCPA', '.UDCPA', 'UTCPA', 
    '.UD', '.UB', '.UK', '.CRI', '.Rf', '.Ra', '.Rs', '.Rp', '.encounter_classification']

# df2 = pd.read_csv("C:/Users/SMY/Desktop/연구실/11. 데이터/06. 2024 내수면 시험 데이터/시험결과 분석용 데이터 정리/202405301159/20240530-115929.csv")

# df2.columns = ['.RD', '.RC', '.K', '.DCPA', '.TCPA', '.UDCPA', 'UTCPA', '.UD', '.UB', '.UK', '.CRI', '.Rf', '.Ra', '.Rs', '.Rp', '.encounter_classification', '.V_opt', '.Collision_cone', '.VO_operate']



# common_values = pd.merge(df1[['time', '.m_nShipID', '.m_fltHeading', '.m_fltVel_U', '.m_fltPos_X', '.m_fltPos_Y', '.DCPA']], 
#                          df2[['.RD', '.RC', '.K', '.DCPA', '.TCPA', '.UDCPA', 'UTCPA', '.UD', '.UB', '.UK', '.CRI', '.Rf', '.Ra', '.Rs', '.Rp', '.encounter_classification', '.Collision_cone', '.V_opt', '.VO_operate']], 
#                          on='.DCPA')



# df_original = common_values.drop_duplicates(subset=['time'])

# # df1[['time', '.m_nShipID', '.m_fltHeading', '.m_fltVel_U', '.m_fltPos_X', '.m_fltPos_Y', 'DCPA']]
# # df2[['RD', 'RC', 'K', 'DCPA', 'TCPA', 'UDCPA', 'UTCPA',' UD', 'UB', 'UK', 'CRI', 'Rf', 'Ra', 'Rs', 'Rp', 'ENC', 'V_opt', 'pub_collision_cone', 'VO_operate']]

df1.to_csv('c:/Users/muyeong/Desktop/simulation_0730/generate_data.csv', index=False)

