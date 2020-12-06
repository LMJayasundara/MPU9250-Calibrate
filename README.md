# MPU9250-Calibrate
Calibrate Gyro, Accel and Mag in MPU9250

<pre>
To check Mag calibrating</br>
  01 - Using Putty to record data.csv
  02 - Download Gnuplot (http://gnuplot.info/)
  03 - Open Gnuplot terminal in csv file dir and type
          set datafile separator ","
          plot "data.csv" using 1:2 title "XY" pointsize 2 pointtype 7, "data.csv" using 1:3 title "XZ" pointsize 2 pointtype 7, "data.csv" using 2:3 title "YZ" pointsize 2 pointtype 7
 </pre>
