import cv2
import glob

image_directory = 'C:/Users/SMY/Desktop/Generate_data/fig/'
image_paths = sorted(glob.glob(image_directory + 'snap*.png'))

def extract_number(filename):
    try:
        return int(''.join(filter(str.isdigit, filename)))
    except ValueError:
        return -1

# 파일 이름에서 숫자를 기준으로 정렬1150
image_paths = sorted(image_paths, key=extract_number)

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('./202405301159.mp4', fourcc, 50.0, (1800,1800))

for image_path in image_paths:
    snap = cv2.imread(image_path)
    print(image_path)
    out.write(snap)

out.release()