"""
Code for the age-weighted method
"""
import sqlite3
import time
from dateutil import parser


def generate_age_counter_weight(data_tuples, total_age_weight):
    """
    Generate counter weight of meta data score to offset age difference
    """
    age_scores = []
    age_weighted_scores = []
    age_diff = []
    for data_tuple in data_tuples:
        time_stamp_string = data_tuple[2]
        time_stamp_number = generate_numeric_timestamp(time_stamp_string)
        age_scores.append(time_stamp_number)
    max_score = max(age_scores) # most recent time stamp
    for score in age_scores:
        age_diff.append(max_score - score)
    age_diff_max = max(age_diff) 
    age_diff_normalized = [age_diff_value/age_diff_max*total_age_weight for age_diff_value in age_diff]
    return age_diff_normalized


def generate_numeric_timestamp(time_stamp_string):
    time_stamp_string = time_stamp_string.replace("_", "-")
    stamps_list = time_stamp_string.split("-")
    date = stamps_list[0] + "-" + stamps_list[1] + "-" + stamps_list[2] + " "
    time_stamp = stamps_list[3] + ":" + stamps_list[4] + ":" + stamps_list[5]
    date_time = date + time_stamp 
    date_obj = parser.parse(date_time)
    return time.mktime(date_obj.timetuple())


def get_top_names(top_num, total_age_weight):
    command = 'SELECT file_name, metadata_score, timestamp FROM Metadata' 
    connection = sqlite3.connect('Editing.db')
    cursor = connection.cursor()
    cursor.execute(command)
    current_top = cursor.fetchall()
    age_counter_weights =generate_age_counter_weight(current_top, total_age_weight)
    weighted_data = []
    for i in range(len(current_top)):
        data_item = current_top[i]
        weighted_item = (data_item[0], data_item[1] - age_counter_weights[i], data_item[2])   
        weighted_data.append(weighted_item)
    sorted_weighted_data = sorted(weighted_data, key=lambda data: data[1], reverse=True)
    top_names = [data[0] for data in sorted_weighted_data]
    return top_names[:top_num]