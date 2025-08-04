
set terminal png size 2200,800 font "Arial,12"
set output 'motor_comparison.png'

set multiplot layout 1,3 title 'Motor Test Data Comparison: Rotor (9 PWM points) vs Rotor with unit (3 PWM points)' font "Arial,16"

# Plot 1: Force Z with curve fitting
set title 'Force Z vs PWM\n(Rotor: continuous, Rotor+unit: 3 discrete points)' font "Arial,13"
set xlabel 'PWM Value' font "Arial,12"
set ylabel 'Force Z (N)' font "Arial,12"
set grid
set key top left font "Arial,10"

# Define fitting functions (polynomial)
f1(x) = a1*x*x + b1*x + c1
f2(x) = a2*x*x + b2*x + c2

# Fit curves
fit f1(x) 'rotor_data.txt' using 1:2 via a1,b1,c1
fit f2(x) 'rotor_with_unit_data.txt' using 1:2 via a2,b2,c2

plot 'rotor_data.txt' using 1:2 with points pt 7 ps 0.6 lc rgb '#0066CC' title 'Rotor (9 PWM levels)', \
     f1(x) with lines lw 3 lc rgb '#004499' title 'Rotor (fitted)', \
     'rotor_with_unit_data.txt' using 1:2 with points pt 9 ps 1.5 lc rgb '#FF3300' title 'Rotor+unit (3 PWM levels)', \
     f2(x) with lines lw 3 lc rgb '#CC1100' title 'Rotor+unit (fitted)'

# Plot 2: Current with enhanced visibility for discrete points
set title 'Current vs PWM\n(Note: Rotor+unit tested only at 3 PWM values)' font "Arial,13"
set xlabel 'PWM Value' font "Arial,12"
set ylabel 'Current (A)' font "Arial,12"
set grid
set key top left font "Arial,10"

# Define fitting functions for current
g1(x) = d1*x*x + e1*x + f1_c
g2(x) = d2*x*x + e2*x + f2_c

# Fit curves
fit g1(x) 'rotor_data.txt' using 1:3 via d1,e1,f1_c
fit g2(x) 'rotor_with_unit_data.txt' using 1:3 via d2,e2,f2_c

# Enhanced visibility - larger points for sparse data
plot 'rotor_data.txt' using 1:3 with points pt 6 ps 0.8 lc rgb '#0066CC' title 'Rotor (continuous)', \
     g1(x) with lines lw 3 lc rgb '#004499' title 'Rotor (fitted)', \
     'rotor_with_unit_data.txt' using 1:3 with points pt 8 ps 2.0 lc rgb '#FF3300' title 'Rotor+unit (3 points)', \
     g2(x) with lines lw 3 lc rgb '#CC1100' title 'Rotor+unit (fitted)'

# Plot 3: RPM with enhanced visibility for discrete points
set title 'RPM vs PWM\n(Rotor+unit: limited PWM test range)' font "Arial,13"
set xlabel 'PWM Value' font "Arial,12"
set ylabel 'RPM' font "Arial,12"
set grid
set key top left font "Arial,10"

# Define fitting functions for RPM
h1(x) = g1_r*x*x + h1_r*x + i1
h2(x) = g2_r*x*x + h2_r*x + i2

# Fit curves
fit h1(x) 'rotor_data.txt' using 1:4 via g1_r,h1_r,i1
fit h2(x) 'rotor_with_unit_data.txt' using 1:4 via g2_r,h2_r,i2

# Enhanced visibility - different point styles
plot 'rotor_data.txt' using 1:4 with points pt 5 ps 0.8 lc rgb '#0066CC' title 'Rotor (9 levels)', \
     h1(x) with lines lw 3 lc rgb '#004499' title 'Rotor (fitted)', \
     'rotor_with_unit_data.txt' using 1:4 with points pt 11 ps 2.0 lc rgb '#FF3300' title 'Rotor+unit (3 levels)', \
     h2(x) with lines lw 3 lc rgb '#CC1100' title 'Rotor+unit (fitted)'

unset multiplot
