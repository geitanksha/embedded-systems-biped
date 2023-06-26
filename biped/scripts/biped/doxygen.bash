#!/usr/bin/env bash

# Go to script directory
cd "$(dirname $0)"

# Source dependencies
source "global.bash"

function main()
{
	# Verify Doxygen
	if [ -d "$project_dir_doxygen/html" ]
	then
		echo "[INFO]: \"$project_name\" Doxygen exists. Quit."
		return
	fi

	# Generate Doxygen
	echo "[INFO]: Generating \"$project_name\" Doxygen..."
	cd "$project_dir"
	doxygen "$project_dir_doxygen/Doxyfile"
	assert_error $? "Failed to generate \"$project_name\" Doxygen"

	echo "[INFO]: Finished generating \"$project_name\" Doxygen."
}

# Call main function
main "$@"
