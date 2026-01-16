#!/bin/bash
DEFAULT_BAM_APP="./BAM_app.exe"
DEFAULT_CORES=4
BAM_APP=${1:-$DEFAULT_BAM_APP}
N=${2:-$DEFAULT_CORES}
FILES="*.bin"
for f in $FILES; do
    (
        BASENAME="$f"
        NUM=${BASENAME//[^0-9]/}
        OUTFILE="bam_out$NUM.mat"
        "$BAM_APP" < "$f" "$OUTFILE"
    ) &

    if [[ $(jobs -r -p | wc -l) -ge $N ]]; then
        wait
    fi
done
wait
echo "all done"
# read -p "Press Enter to continue..."
