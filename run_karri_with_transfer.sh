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
scriptName=karri_transfers

# Lies Eingabeparameter
karriSourceDir=$1
inputDir=$2
instanceName=$3
outputBaseDir=$4
vehicles=$5
requests=$6

# Prüfe, ob Output Directory existiert
if ! [ -d "$outputBaseDir" ]; then
	echo "Output directory ${outputBaseDir} does not exist."
	exit 1
fi

echo "Performing karri-with-transfers experiments."

# Pfade zu Inputs
vehGraph=$inputDir/Graphs/${instanceName}_pedestrian_veh.gr.bin
psgGraph=$inputDir/Graphs/${instanceName}_pedestrian_psg.gr.bin
vehCh=$inputDir/CHs/${instanceName}_pedestrian_veh_time.ch.bin
psgCh=$inputDir/CHs/${instanceName}_pedestrian_psg_time.ch.bin

echo $vehGraph
echo $psgGraph
echo $vehCh
echo $psgCh

vehiclePath=$inputDir/Vehicles/${instanceName}-${vehicles}.csv
requestPath=$inputDir/Requests/${instanceName}-${requests}.csv

echo $vehiclePath
echo $requestPath

# Erzeuge konkretes Output-Directory, dessen Name aus scriptName (s.o.) + instanceName + aktuellem timestamp besteht.
currentTime=$(date "+%m.%d-%H:%M")
karriOutputDir=$outputBaseDir/karri-with-transfers/${scriptName}_${instanceName}_v-${vehicles}_r-${requests}_${currentTime}
mkdir -p $karriOutputDir
mkdir -p $karriOutputDir/wt
mkdir -p $karriOutputDir/wot

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
cmake --build $karriBinaryDir --target karri --clean-first -j 16


# Lasse KaRRi mit Transfers laufen
echo $karriBinaryDir


karriOutputDirWot=$karriOutputDir/wot/wot
karriOutputDirWt=$karriOutputDir/wt/wt

printf "Running without transfers...\n"

$karriBinaryDir/Launchers/karri -trans 0 -w 300 -p-radius 0 -d-radius 0 -veh-g $vehGraph -psg-g $psgGraph -v $vehiclePath -r $requestPath -veh-h $vehCh -psg-h $psgCh -o $karriOutputDirWot

printf "\nRunning with transfers...\n"
$karriBinaryDir/Launchers/karri -trans 1 -w 300 -p-radius 0 -d-radius 0 -veh-g $vehGraph -psg-g $psgGraph -v $vehiclePath -r $requestPath -veh-h $vehCh -psg-h $psgCh -o $karriOutputDirWt