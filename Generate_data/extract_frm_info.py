import pandas as pd
import csv

input_file_path = "C:/Users/wonjun/Desktop/Generate_data/YOO/inha_scenario(crossing_wpchange)_2024-07-31-12-25-40-frm_info.csv"
output_file_path = "C:/Users/wonjun/Desktop/Generate_data/YOO/inha_scenario(crossing_wpchange)_2024-07-31-12-25-40-frm_info_pre.csv"

data = pd.read_csv(input_file_path)

selected_data = ['time', '.m_nShipID', '.m_fltHeading', '.m_fltVel_U', '.m_fltPos_X', '.m_fltPos_Y']

frm_info_data = data[selected_data]

frm_info_data.to_csv(output_file_path, index=False)