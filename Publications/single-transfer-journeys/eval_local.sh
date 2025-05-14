# Move the necessary files from the output folder

instanceName=$1
outputName=$2

sourceDir=../../Outputs
targetDir=./inputs/$instanceName/$outputName/
outputDir=./outputs/$instanceName/$outputName/

echo "Evaluating local results of karri run: ${instanceName}"

echo "Copying the necessary outputs files....."
mkdir -p $targetDir
cp $sourceDir/$instanceName.perf_transf_als_dveh.csv $targetDir
cp $sourceDir/$instanceName.perf_transf_als_pveh.csv $targetDir

mkdir -p $outputDir

python3 transfer_als.py $instanceName $targetDir $outputDir