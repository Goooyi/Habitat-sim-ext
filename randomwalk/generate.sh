#!/bin/bash

# Set variables
get_seeded_random()
{
  seed="$1"
  openssl enc -aes-256-ctr -pass pass:"$seed" -nosalt \
    </dev/zero 2>/dev/null
}

RANDOM=42
scene_folder="/home/gao/dev/project_remote/Habitat-sim-ext/randomwalk/data/hm3d-val-glb-v0.2"
objects=("angry_girl" "ferbibliotecario" "miniature_cat" "robot_2020" "shiba" "toy_car")
# objects=("big_bot" "tsai_ing-wen" "noirnwa" "big_bot" "tsai_ing-wen" "noirnwa")
# objects=("big_bot" "tsai_ing-wen" "noirnwa")
num_loops=3
num_pedestrians=(3 6)
speed_values=(1 2 3)
max_run_time=100
base_seed=1
seed=42
# base_seed=72
# seed=73
# base_seed=23
# seed=24

# Loop through scene files
# i=0
# loop for 30 scenes
# for scene_file in $(find "$scene_folder" -name "*.glb" | shuf --random-source=<(echo 42) -e | head -n 30); do
files=($(find "$scene_folder" -name "*.glb"))
echo $files
for scene_file in $(find "$scene_folder" -name "*.glb" | shuf --random-source=<(get_seeded_random 42)| head -n 2); do
# for scene_file in $(shuf --random-source=<(echo 42) -e "${files}" | head -n 10 ); do
  # i=$((i+1))
  seed=$base_seed

  # Loop through object choices
  for k in $(seq 1 ${#objects[@]}); do
    if [[ $k -eq 1 ]]; then
      # object=${objects[$((RANDOM%${#objects[@]}))]}
      for object in ${objects[@]}; do
        for l in ${num_pedestrians[@]}; do
          pedestrians=$(printf "%s " $(for m in $(seq 1 $l); do printf "$object "; done))
          for speed in ${speed_values[@]}; do
            sleep 1
            video_prefix="scene_${scene_file##*/}_object_${object}_objCount_${l}_speed_${speed}"
            echo "run randomwalk.py with seed=$seed, pedestrians=$pedestrians"
            timeout ${max_run_time} python ./randomwalk/random_walk.py --scene "$scene_file" --pedestrians $pedestrians --speed $speed --seed $seed --non-dynamic --video-prefix $video_prefix
            # timeout ${max_run_time} python ./randomwalk/random_walk.py --scene "$scene_file" --pedestrians $pedestrians --speed $speed --seed $seed --video-prefix $video_prefix
            while [ $? -ne 0 ]; do
              seed=$((seed+1))
              echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
              echo "Command returned errors or took too long, rerunning with seed=$seed, pedestrians=$pedestrians"
              sleep 1
              video_prefix="scene_${scene_file##*/}_object_${object}_objCount_${l}_speed_${speed}"
              timeout ${max_run_time} python ./randomwalk/random_walk.py --scene "$scene_file" --pedestrians $pedestrians --speed $speed --seed $seed --non-dynamic --video-prefix $video_prefix
              # timeout ${max_run_time} python ./randomwalk/random_walk.py --scene "$scene_file" --pedestrians $pedestrians --speed $speed --seed $seed --video-prefix $video_prefix
            done
          done
        done
      done
    elif [[ $k -eq 3 ]]; then
      # object_1=${objects[$((RANDOM%${#objects[@]}))]}
      # object_2=${objects[$((RANDOM%${#objects[@]}))]}
      # object_3=${objects[$((RANDOM%${#objects[@]}))]}
      for i in $(seq 1 3); do
        selected_objects=($(shuf --random-source=<(get_seeded_random 42) -n 3 -e "${objects[@]}"))
        object_1=${selected_objects[0]}
        object_2=${selected_objects[1]}
        object_3=${selected_objects[2]}
        while [ "$object2" == "$object1" ]; do
          object2="${objects[$RANDOM % ${#objects[@]}]}"
        done
        for l in ${num_pedestrians[@]}; do
          pedestrians=$(printf "%s " $(for m in $(seq 1 $((l/3))); do printf "$object_1 $object_2 $object_3 "; done))
          for speed in ${speed_values[@]}; do
            sleep 1
            video_prefix="scene_${scene_file##*/}_object_${object_1}_${object_2}_${object_3}_objCount_${l}_speed_${speed}"
            echo "run randomwalk.py with seed=$seed, pedestrians=$pedestrians"
            timeout ${max_run_time} python ./randomwalk/random_walk.py --scene "$scene_file" --pedestrians $pedestrians --speed $speed --seed $seed --non-dynamic --video-prefix $video_prefix
            # timeout ${max_run_time} python ./randomwalk/random_walk.py --scene "$scene_file" --pedestrians $pedestrians --speed $speed --seed $seed --video-prefix $video_prefix
            while [ $? -ne 0 ];do
              seed=$((seed+1))
              echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
              echo "Command returned errors or took too long, rerunning with seed=$seed, pedestrians=$pedestrians"
              sleep 1
              video_prefix="scene_${scene_file##*/}_object_${object_1}_${object_2}_${object_3}_objCount${l}_speed_${speed}"
              timeout ${max_run_time} python ./randomwalk/random_walk.py --scene "$scene_file" --pedestrians $pedestrians --speed $speed --seed $seed --non-dynamic --video-prefix $video_prefix
              # timeout ${max_run_time} python ./randomwalk/random_walk.py --scene "$scene_file" --pedestrians $pedestrians --speed $speed --seed $seed --video-prefix $video_prefix
            done
          done
        done
      done
    fi
  done
done

