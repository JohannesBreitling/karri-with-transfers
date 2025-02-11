#!/bin/bash


# Skript zum Ausführen von KaRRi auf Compute-Servern.
# Verwendung: sbatch --partition=<compute-name> ./run_karri_base.sh <source-dir> <input-dir> <instance-name> <output-base-dir> [num runs]
#	- <compute-name> : Name der Maschine, z.B. 'cook'
#	- <source-dir> : absoluter Pfad zum untersten Ordner deines Repository, also wahrscheinlich sowas wie /nfs/home/jbreitling/karri/
#	- <input-dir> : absoluter Pfad zu Inputs Ordner (darin werden Ordner 'Graphs', 'Vehicles', 'CHs' für die Inputdateien erwartet)
# 	- <instance-name> : entweder Berlin-1pct oder Berlin-10pct
#	- <output-base-dir> : absoluter Pfad zu frei gewähltem Output-Ordner, z.B. /nfs/home/jbreitling/Outputs/ (Ordner muss bereits vor Aufruf existieren)
#	- [num runs]: optionale Angabe für Anzahl Runs. Wichtig für Laufzeitmessungen. Default: 1

# Name des Skripts (zur Identifizierung)
scriptName=run_karri_with_transfers

# Lies Eingabeparameter
karriSourceDir=$1
inputDir=$2
instanceName=$3
outputBaseDir=$4
num_runs=$5

# Prüfe, ob Output Directory existiert
if ! [ -d "$outputBaseDir" ]; then
	echo "Output directory ${outputBaseDir} does not exist."
	exit 1
fi

# Setze num_runs auf 1, falls nicht anders angegeben
[ -z ${num_runs} ] && num_runs=1
echo "Performing ${num_runs} runs."


# Pfade zu Inputs

instanceName=${instanceName}_pedestrian

vehGraph=$inputDir/Graphs/${instanceName}_veh.gr.bin
psgGraph=$inputDir/Graphs/${instanceName}_psg.gr.bin
vehicles=$inputDir/Vehicles/$instanceName.csv
requests=$inputDir/Requests/$instanceName.csv
vehCh=$inputDir/CHs/${instanceName}_veh_time.ch.bin
psgCh=$inputDir/CHs/${instanceName}_psg_time.ch.bin

# Erzeuge konkretes Output-Directory, dessen Name aus scriptName (s.o.) + instanceName + aktuellem timestamp besteht.
currentTime=$(date "+%Y.%m.%d-%H:%M")
karriOutputDir=$outputBaseDir/KaRRi/${scriptName}_${instanceName}_${currentTime}
mkdir -p $karriOutputDir

# Baue KaRRi nach $karriSourceDir/Build/Release.
# Konfiguriere KaRRi CMake compile time parameter.
karriBinaryDir=$karriSourceDir/Build/Release
dependencyInstallDir=/global_data/laupichler/KaRRi/install
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="${dependencyInstallDir}"\
	-DKARRI_ELLIPTIC_BCH_USE_SIMD=ON -DKARRI_ELLIPTIC_BCH_LOG_K=4 \
	-DKARRI_PALS_STRATEGY=COL -DKARRI_PALS_USE_SIMD=ON -DKARRI_PALS_LOG_K=3 \
	-DKARRI_DALS_STRATEGY=COL -DKARRI_DALS_USE_SIMD=ON -DKARRI_DALS_LOG_K=3 \
	-DKARRI_PD_DISTANCES_USE_SIMD=ON -DKARRI_PD_DISTANCES_LOG_K=5 \
	-DKARRI_PSG_COST_SCALE=1 \
	-DKARRI_VEH_COST_SCALE=1 \
	-S $karriSourceDir -B $karriBinaryDir
cmake --build $karriBinaryDir --target karri -j 16


# Lasse KaRRi 5 Mal laufen (nur relevant für Laufzeitmessungen, für Qualität reicht ein Run)
for (( i = 1; i <= ${num_runs}; i++ )) 
do
	# Run pedestrian/KaRRi, radius 300, wait time 300
	# ID, um zwischen 5 runs zu unterscheiden
	run_id=${scriptName}_${instanceName}_run$i
	echo "Running run: ${run_id}"
	echo $karriBinaryDir
	$karriBinaryDir/Launchers/karri -w 300 -p-radius 0 -d-radius 0 -veh-g $vehGraph -psg-g $psgGraph -v $vehicles -r $requests -veh-h $vehCh -psg-h $psgCh -o $karriOutputDir/$run_id
done

