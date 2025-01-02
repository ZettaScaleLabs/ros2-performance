#!/usr/bin/env bash

TEST_PREFIX=""
if [ -z "${RMW_IMPLEMENTATION}" ];
    echo "RMW_IMPLEMENTATION env var is empty!"
else
    TEST_PREFIX="${RMW_IMPLEMENTATION}_"
    echo "Running memory benchmark with RMW: ${RMW_IMPLEMENTATION}"
fi

current_date=$(date +"%d_%m_%y_%Hh%M")
results_dir="${TEST_PREFIX}memory_${current_date}"
mkdir -p "$results_dir"
echo "Storing results in $results_dir"

ros2 run memory_benchmark default_nodes > $results_dir/${TEST_PREFIX}default_nodes.csv
ros2 run memory_benchmark nodes_params_off > $results_dir/${TEST_PREFIX}nodes_params_off.csv
ros2 run memory_benchmark default_subs_params_off > $results_dir/${TEST_PREFIX}default_subs_params_off.csv
ros2 run memory_benchmark default_pubs_params_off > $results_dir/${TEST_PREFIX}default_pubs_params_off.csv
ros2 run memory_benchmark default_clients_params_off > $results_dir/${TEST_PREFIX}default_clients_params_off.csv
ros2 run memory_benchmark default_services_params_off > $results_dir/${TEST_PREFIX}default_services_params_off.csv
ros2 run memory_benchmark nodes_params_off_logging_on > $results_dir/${TEST_PREFIX}nodes_params_off_logging_on.csv
ros2 run memory_benchmark pub_sub_diff_topic > $results_dir/${TEST_PREFIX}pub_sub_diff_topic.csv
ros2 run memory_benchmark pub_sub_same_topic > $results_dir/${TEST_PREFIX}pub_sub_same_topic.csv
ros2 run memory_benchmark pub_sub_diff_msg_type > $results_dir/${TEST_PREFIX}pub_sub_diff_msg_type.csv
ros2 run memory_benchmark cli_serv_diff_topics > $results_dir/${TEST_PREFIX}cli_serv_diff_topics.csv
ros2 run memory_benchmark cli_serv_same_topics > $results_dir/${TEST_PREFIX}cli_serv_same_topics.csv
ros2 run memory_benchmark pub_sub_big_history_size > $results_dir/${TEST_PREFIX}pub_sub_big_history_size.csv
ros2 run memory_benchmark pub_sub_big_message_size > $results_dir/${TEST_PREFIX}pub_sub_big_message_size.csv
