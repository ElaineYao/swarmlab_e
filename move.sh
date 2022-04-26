#!/bin/bash

mkdir ./results/$1
mv pos_ned_1.csv pos_ned_2.csv pos_ned_3.csv pos_ned_4.csv pos_ned_5.csv ./results/$1

mv dis_agents.csv ./results/$1

mv dis_obs.csv ./results/$1

mv vel_xyzt_1.csv vel_xyzt_2.csv vel_xyzt_3.csv vel_xyzt_4.csv vel_xyzt_5.csv ./results/$1
