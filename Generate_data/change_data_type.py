import pandas as pd
import numpy as np
import ast
# from Inha_DataProcess import Inha_dataProcess

df = pd.read_csv("C:/Users/SMY/Desktop/연구실/11. 데이터/06. 2024 내수면 시험 데이터/rosbag_to_csv/202405291434/frm_info_pre.csv")

# 문자열을 리스트로 변환하는 함수 정의
def convert_to_list(val):
    try:
        if isinstance(val, str):
            # 'nan'을 'None'으로 변환
            val = val.replace('nan', 'None')
            val = ast.literal_eval(val)
            # None을 np.nan으로 변환
            return [np.nan if v is None else v for v in val]
        return val
    except (ValueError, SyntaxError):
        return val
    
# 문자열을 튜플로 변환하는 함수 정의
def convert_to_tuple(val):
    try:
        if isinstance(val, str):
            # 'nan'을 np.nan으로 변환
            val = val.replace('nan', 'None')
            val = ast.literal_eval(val)
            # None을 np.nan으로 변환
            return tuple(np.nan if v is None else v for v in val)
        return val
    except (ValueError, SyntaxError):
        return val

df['.m_nShipID'] = df['.m_nShipID'].apply(convert_to_list)

columns_to_convert = ['.m_fltHeading', '.m_fltVel_U', '.m_fltPos_X', '.m_fltPos_Y']
for col in columns_to_convert:
    df[col] = df[col].apply(convert_to_tuple)



