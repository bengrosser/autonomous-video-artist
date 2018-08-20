import sys
import time
import numpy as np
import cv2
from numba import jit


def create_grid_flip(img):
    row_size, col_size = img.shape
    grid = np.zeros((2 * row_size, 2 * col_size))
    grid[0:row_size, 0:col_size] = img
    img_hori = cv2.flip(grid, 0)
    img_left = cv2.bitwise_or(img_hori, grid)
    img_right = cv2.flip(img_left, 1)
    result_image = cv2.bitwise_or(img_left, img_right)
    return result_image


def fft(img):
    # x, y = img.shape
    # This dft has all the values we need for the recovery of the photo, I hope
    dft = cv2.dft(np.float32(img), flags=cv2.DFT_COMPLEX_OUTPUT)
    # dft = np.fft.fftshift(dft) # I actually don't want to have it shift
    # dft_shift = np.fft.fftshift(dft)
    result_magnititude = 20 * np.log(cv2.magnitude(dft[:, :, 0], dft[:, :, 1]))
    result_magnititude = np.uint8(result_magnititude)
    # result_color_magnititude = cv2.cvtColor(result_magnititude, cv2.COLOR_GRAY2RGB)
    # cv2.imwrite("wc_frequency_magnitude.jpg", result_color_magnititude)
    return dft, result_magnititude


@jit(nopython=True)
def synthesis_with_jit(x_freq, y_freq, complex_a, complex_b, row_size, col_size, x_data, y_data):
    result_rows, result_cols =  x_data.shape
    z_result = np.zeros((result_rows, result_cols))
    for i in range(result_rows):
        for j in range(result_cols):
            z_value = complex_a*(np.cos(x_freq*x_data[i][j]))*(np.cos(y_freq*y_data[i][j])) + \
                     complex_b*(np.sin(x_freq*x_data[i][j]))*(np.sin(y_freq*y_data[i][j]))
            z_result[i][j] = z_value
    # z_result = complex_a*(np.cos(x_freq*x_data))*(np.cos(y_freq*y_data)) + \
    #                   complex_b*(np.sin(x_freq*x_data))*(np.sin(y_freq*y_data))
    return z_result


def synthesis(x_freq, y_freq, complex_a, complex_b, row_size, col_size):
    X_data = np.arange(row_size)
    Y_data = np.arange(col_size)
    X_data, Y_data = np.meshgrid(X_data, Y_data)
    '''
    This line can make a symmetry of the input images
    '''
    # Z_data_original = complex_a*(np.cos(x_freq*X_data))*(np.cos(y_freq*Y_data)) + \
    #         complex_b*(np.sin(x_freq*X_data))*(np.sin(y_freq*Y_data))
    Z_data = synthesis_with_jit(x_freq, y_freq, complex_a, complex_b, row_size, col_size, X_data, Y_data)
    return Z_data


def generate_wave(img, limit_number):
    """
    Based upon the input image and limit number, generate restored fourier transform 3D wave
    :param img: Input image
    :param limit_number: How many individual waves we need to combine
    :return: Generated waves
    """
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    grid_result = create_grid_flip(img)
    complex_dft_result, result_magnitude = fft(grid_result)
    x, y, _ = complex_dft_result.shape
    Z_value = np.zeros((y, x))
    for i in range(limit_number):
        i_index = int(i) / 20
        j_index = int(i) % 20
        x_freq = 2 * np.pi * (i_index / float(x))
        # progress = 100 * (float(i_index * x) / float(x * y))
        y_freq = 2 * np.pi * (j_index / float(y))

        if ((i_index == 0) and (j_index == 0)) or ((i_index == ((x / 2) - 1)) and ((j_index == ((y / 2) - 1)))):
            complex_a = complex_dft_result[i_index][j_index][0] / (x)
            complex_b = -complex_dft_result[i_index][j_index][1] / (y / 2)
        else:
            complex_a = complex_dft_result[i_index][j_index][0] / (x / 2)
            complex_b = -complex_dft_result[i_index][j_index][1] / (y / 2)
            # complex_a = complex_dft_result[i][j][0]
            # complex_b = complex_dft_result[i][j][1]
        # phase = np.arctan2(complex_b, complex_a)
        temp_result = synthesis(x_freq, y_freq, complex_a, complex_b, x, y)
        Z_value = Z_value + temp_result

    Z_value = Z_value / float(x*y)
    Z_row, Z_col = Z_value.shape
    result = Z_value[0:(Z_row / 2), 0:(Z_col / 2)]
    return result


def easy_visualize(frame):
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    f = np.fft.fft2(img)
    fshift = np.fft.fftshift(f)
    magnitude_spectrum = 20 * np.log(np.abs(fshift))
    magnitude_spectrum = np.rint(magnitude_spectrum)
    result = np.uint8(magnitude_spectrum)
    result = cv2.cvtColor(result, cv2.COLOR_GRAY2RGB)
    return result


def produce_wave_video(src, output, frame_rate, res1, res2):
    src_path = "./videos/" + src
    camera = cv2.VideoCapture(src_path)
    # fourcc = cv2.VideoWriter_fourcc(*'XVID')
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    resolution = (int(res1), int(res2))
    out = cv2.VideoWriter(output, fourcc, frame_rate, resolution)
    start_time = time.time()
    while True:
        grabbed, frame = camera.read()
        if grabbed:
            visualized_result = easy_visualize(frame)
            out.write(visualized_result)
        else:
            break
    end_time = time.time()
    camera.release()
    out.release()
    cv2.destroyAllWindows()
