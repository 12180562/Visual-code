import pandas as pd
import matplotlib.pyplot as plt


# CSV 파일 읽기
data = pd.read_csv('C:/Users/SMY/Desktop/연구실/11. 데이터/06. 2024 내수면 시험 데이터/시험결과 분석용 데이터 정리/202405291616/common_values.csv')

# .VO_operate 열 처리
def convert_vo_operate(value):
    if value == 'TRUE':
        return 1
    else:
        return 0
    


def plot_vo_operate(time, vo_operate_values, save_path):
    time_x = list(range(1, time+1, 1))
    if len(vo_operate_values) == 1:
        vo_operate_y = vo_operate_values * time
    else:
        vo_operate_y = [1 if val else 0 for val in vo_operate_values]

    vo_operate_ya = vo_operate_values
    vo_operate_yb = []

    for i in range(len(time_x)-len(vo_operate_ya)):
        vo_operate_yb.append(0)
    
    vo_operate_y = vo_operate_ya + vo_operate_yb

    
    fig, ax1 = plt.subplots()

    ax1.plot(time_x, vo_operate_y, 'b-', label='VO Operate')
    ax1.fill_between(time_x[0:], vo_operate[0:], color='r', alpha=0.1)

    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('VO Operate (1: True, 0: False)')
    ax1.set_ylim(-0.1, 1.1)
    ax1.set_yticks([0, 1])

    # 범례 설정
    ax1.legend(loc='best')

    plt.savefig(save_path)

    # 그래프 출력
    plt.show()



data['.VO_operate'] = data['.VO_operate'].apply(convert_vo_operate)

print(data['.VO_operate'])

# # 데이터 길이
# data_len = len(data)

# # 필요한 리스트 초기화
# vo_operate_list = []

# # 데이터를 순회하며 그래프 생성 및 저장
# for i in range(data_len):
#     # 데이터 추출 (예시: 데이터는 적절한 방식으로 추출)
#     vo_operate = data['.VO_operate'].iloc[i]
#     vo_operate_values = vo_operate_list.append(vo_operate)

#     # 시간 길이 (예시: 고정된 값 또는 데이터에 따라 결정)
#     time_length = data_len # 예시로 고정된 시간 길이 사용

#     # 저장 경로 설정
#     save_path = f'C:/Users/SMY/Desktop/연구실/11. 데이터/06. 2024 내수면 시험 데이터/시험결과 분석용 데이터 정리/202405291616/figvo/snap{i}.png'

#     # 그래프 생성 및 저장
#     plot_vo_operate(time_length, vo_operate_values, save_path)

#     print(f'그래프 {i} 저장 완료')




