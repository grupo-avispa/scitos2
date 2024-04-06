#!/bin/bash

###############################################################################

confirm()
{
	echo -n "$1 ? (n) "
	read ans
	case "$ans" in
	y|Y|yes|YES|Yes) return 0 ;;
	*) return 1 ;;
	esac
}

infoMsg()
{
	echo "$1" | tee -a $LOG_FILE
}

fatalInfoMsg()
{
	echo $1 >> $LOG_FILE
	echo ""
	echo "....."
	tail -n20 $LOG_FILE
	echo ""
	echo "$1 For more details look into $LOG_FILE"
	exit -1
}

installPackage()
{
	infoMsg "INFO: Install $1..."
	$BOOTSTRAP_DIR/bin/mirapackage --noninteractive -P $INSTALL_DIR -I $1 2>&1 | tee -a $LOG_FILE
	if [ $PIPESTATUS -ne 0 ] ; then
		fatalInfoMsg "FATAL: Install $1 failed."
	fi
}

FTPURL=ftp://mira-project.org

POSITIONAL_ARGS=()

while [[ $# -gt 0 ]]; do
    case $1 in
    -s | --system)
        SYSTEM="$2"
        shift # past argument
        shift # past value
        ;;
    -d | --destination)
        DEST="$2"
        shift # past argument
        shift # past value
        ;;
    -h | --help)
        HELP=YES
        shift # past argument
        ;;
    -* | --*)
        echo "Unknown option $1"
        exit 1
        ;;
    *)
        POSITIONAL_ARGS+=("$1") # save positional arg
        shift # past argument
        ;;
    esac
done

if [ ! -z "$POSITIONAL_ARGS" ] && [ -z "$SYSTEM" ] ; then
	SYSTEM=${POSITIONAL_ARGS[0]}
fi

###############################################################################

echo ""
echo "************************************************************************"
echo "*"
echo "* This is the MIRA binary installer. It will download and compile a"
echo "* basic configuration of MIRA on your system."
echo "*"
echo "************************************************************************"
echo ""

if [ -z "$SYSTEM" ] || [ ! -z "$HELP" ] ; then
	echo "Usage: $0 -s systemName [-d destinationDir]"
	echo "  Following systems are supported: "
	echo "    redhat-el7-x64      : RedHat Enterprise Linux / CentOS 7.x, 64bit"
	echo "    redhat-el8-x64      : RedHat Enterprise Linux / CentOS / Oracle Linux 8.x, 64bit"
	echo "    ubuntu-1804lts-x64  : Ubuntu 18.04LTS, 64bit"
	echo "    ubuntu-2004lts-x64  : Ubuntu 20.04LTS, 64bit"
	echo "    ubuntu-2204lts-x64  : Ubuntu 22.04LTS, 64bit"
	exit
fi

#echo "System type       = ${SYSTEM}"
#echo "Installation path = ${DEST}"

###############################################################################

INSTALL_DIR_DEFAULT=`pwd`/mira

if [ -z "$DEST" ] ; then
	read -p "Please input the installation directory. Enter=default($INSTALL_DIR_DEFAULT): " DEST
fi

if [ -z "$DEST" ] ; then
	DEST=$INSTALL_DIR_DEFAULT
	echo "Install into: $DEST"
fi

# normalize the path
INSTALL_DIR=$(cd $(eval "dirname ${DEST}");pwd)/$(eval "basename ${DEST}")

#######################################################################
# Variables

BOOTSTRAP_DIR=$INSTALL_DIR/bootstrap

DOWNLOAD_DIR=$INSTALL_DIR
LOG_FILE="$INSTALL_DIR/install.log"
STAGE_FILE="$INSTALL_DIR/install.stage"
START_STAGE=1
END_STAGE=12

MIRAENV_URL="${FTPURL}/repos/MIRA-main/${SYSTEM}/env/latest/MIRA-MIRAenvironment*.zip"
MIRAEXT_URL="${FTPURL}/repos/MIRA-main/${SYSTEM}/external/latest/MIRA-external*.zip"
MIRABASE_URL="${FTPURL}/repos/MIRA-main/${SYSTEM}/base/latest/MIRA-MIRABase*.zip"
MIRAPACKAGE_URL="${FTPURL}/repos/MIRA-main/${SYSTEM}/tools/mirapackage/latest/MIRA-MIRAPackage*.zip"

#######################################################################

# check if the directory already exists
if [ -d $INSTALL_DIR ] ; then
	#exists check for install.stage file
	if [ -e $STAGE_FILE ] ; then
		# stage file exists retrieve last stage
		read -r LAST_STAGE < $STAGE_FILE
		if [ -n $LAST_STAGE ] ; then
			echo ""
			if confirm "The Directory $INSTALL_DIR already exists and contains a previously started installation. Continue this installation?" ; then 
				START_STAGE=$LAST_STAGE
			else
				echo "Removing old directory content..."
				rm -rf $INSTALL_DIR
			fi
		else
			echo ""
			if confirm "Directory $INSTALL_DIR already exists. All data in that directory will be lost! Continue?" ; then
				echo "Removing old directory content..."
				rm -rf $INSTALL_DIR
			else
 				exit
			fi
		fi
	else
		echo ""
		if confirm "Directory $INSTALL_DIR already exists. All data in that directory will be lost! Continue?" ; then
			echo "Removing old directory content..."
			rm -rf $INSTALL_DIR
		else
 			exit
		fi
	fi
fi

echo ""

for (( i=$START_STAGE; i<=$END_STAGE; i++ ))
do
	case $i in
	1)
		#######################################################################
		# Create directory

		echo "INFO: Create install directory..."

		# create the directory
		mkdir -p $INSTALL_DIR
		if [ $? -ne 0 ] ; then
			echo "FATAL: Failed to create directory $INSTALL_DIR. Abort!"
			exit -1
		fi

		# create a bootstrap directory
		mkdir -p $BOOTSTRAP_DIR

		# Create log file
		touch $LOG_FILE
		touch $STAGE_FILE;;
	2)
		#######################################################################
		# Download MIRAEnvironment

		infoMsg "INFO: Downloading MIRAEnvironment..."
		wget -q $MIRAENV_URL -P $DOWNLOAD_DIR
		if [ $? -ne 0 ] ; then
			fatalInfoMsg "FATAL: Failed to download $MIRAENV_URL"
		fi;;
	3)
		#######################################################################
		# Download MIRA external

		infoMsg "INFO: Downloading MIRA external..."
		wget -q $MIRAEXT_URL -P $DOWNLOAD_DIR
		if [ $? -ne 0 ] ; then
			fatalInfoMsg "FATAL: Failed to download $MIRAEXT_URL"
		fi;;
	4)
		#######################################################################
		# Download MIRABase

		infoMsg "INFO: Downloading MIRABase..."
		wget -q $MIRABASE_URL -P $DOWNLOAD_DIR
		if [ $? -ne 0 ] ; then
			fatalInfoMsg "FATAL: Failed to download $MIRABASE_URL"
		fi;;
	5)
		#######################################################################
		# Download MIRAPackage

		infoMsg "INFO: Downloading MIRAPackage..."
		wget -q $MIRAPACKAGE_URL -P $DOWNLOAD_DIR
		if [ $? -ne 0 ] ; then
			fatalInfoMsg "FATAL: Failed to download $MIRAPACKAGE_URL"
		fi;;
	6)
		#######################################################################
		# Create the MIRA bootstrap environment

		infoMsg "INFO: Unzip MIRAenvironment..."
		# unzip MIRAenvironment
		cd $BOOTSTRAP_DIR
		unzip -q $DOWNLOAD_DIR/MIRA-MIRAenvironment*.zip

		# Clean up
		rm $DOWNLOAD_DIR/MIRA-MIRAenvironment*.zip;;
	7)
		#######################################################################
		# Unpack MIRA external in bootstrap environment
		
		infoMsg "INFO: Unzip MIRA external..."
		# unzip MIRABase
		cd $BOOTSTRAP_DIR
		unzip -q $DOWNLOAD_DIR/MIRA-external*.zip

		# Clean up
		rm $DOWNLOAD_DIR/MIRA-external*.zip;;
	8)
		#######################################################################
		# Unpack MIRABase in bootstrap environment

		infoMsg "INFO: Unzip MIRABase..."
		# unzip MIRABase
		cd $BOOTSTRAP_DIR
		unzip -q $DOWNLOAD_DIR/MIRA-MIRABase*.zip

		# Clean up
		rm $DOWNLOAD_DIR/MIRA-MIRABase*.zip;;
	9)
		#######################################################################
		# Unpack MIRAPackage in bootstrap environment

		infoMsg "INFO: Unzip MIRAPackage..."
		# unzip MIRAPackage
		cd $BOOTSTRAP_DIR/tools
		unzip -q $DOWNLOAD_DIR/MIRA-MIRAPackage*.zip

		# Clean up
		rm $DOWNLOAD_DIR/MIRA-MIRAPackage*.zip;;
	10)
		#######################################################################
		# Fixing directory names

		infoMsg "INFO: Fixing directory names in MIRA base system."
		cd $BOOTSTRAP_DIR

		for p in `find . -type d -a -name "MIRA_*"`; do
			b=`basename $p`
			newDir=${b/MIRA_/}
			if [ ! -d $newDir ] ; then mkdir $newDir ; fi
			mv $p/* $newDir
			rmdir $p
		done

		# Delete package files in BOOTSTRAP directory to avoid the mirapackage
		# will find them.
		find $BOOTSTRAP_DIR -name "*.package" | xargs rm -f

		;;
	11)
		#######################################################################
		# Use mirapackage to download and install the rest of MIRA

		cd $BOOTSTRAP_DIR
		export MIRA_PATH="$BOOTSTRAP_DIR:$INSTALL_DIR"
		export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"$BOOTSTRAP_DIR/lib"

		if [ -f ${HOME}/.config/mira/mirapackage.xml ] ; then
			miraPkgBackup=${HOME}/mirapackage_$(date +%F_%H-%M-%S).xml
			infoMsg "Info: An existing configuration of mirapackage was found. Moved backup to ${miraPkgBackup} and will create a clean environment"
			mv ${HOME}/.config/mira/mirapackage.xml ${miraPkgBackup}
		fi

		infoMsg "INFO: Add installation repository"
		$BOOTSTRAP_DIR/bin/mirapackage --addurl "${FTPURL}/repos/MIRA-main/${SYSTEM}/MIRA-main.repo" 2>&1 | tee -a $LOG_FILE
		if [ $PIPESTATUS -ne 0 ] ; then
			fatalInfoMsg "FATAL: Add installation repository failed."
		fi

		infoMsg "INFO: Reindex installation repository"
		$BOOTSTRAP_DIR/bin/mirapackage --reindex 2>&1 | tee -a $LOG_FILE
		if [ $PIPESTATUS -ne 0 ] ; then
			fatalInfoMsg "FATAL: Reindex installation repository failed."
		fi

		installPackage MIRAenvironment
		installPackage external
		installPackage MIRABase
		installPackage MIRAPackage

		installPackage MIRAFramework
		installPackage GUIWidgets

		installPackage CommonCodecs
		installPackage CommonVisualization
		installPackage PlotVisualization
		installPackage Localization
		installPackage Navigation
		installPackage RigidModel
		installPackage RobotDataTypes
		installPackage VideoCodecs
		installPackage GraphVisualization
		installPackage CameraParameters

		installPackage MIRACenter
		installPackage TapeEditor

		installPackage MIRA
		installPackage MIRAgui
		installPackage MIRAtape
		installPackage MIRAinspect
		installPackage MIRAWizard

		# Now delete the bootstrap directory
		rm -rf $BOOTSTRAP_DIR

		;;
	12)
		#######################################################################
		# Create bash file

		START_BASH=$INSTALL_DIR/start.bash
		touch $START_BASH
		echo "export MIRA_PATH=$INSTALL_DIR" >> $START_BASH
		echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:$INSTALL_DIR/lib" >> $START_BASH
		echo "export PATH=\$PATH:$INSTALL_DIR/bin" >> $START_BASH
		echo "source $INSTALL_DIR/scripts/mirabash" >> $START_BASH;;
	esac
	# Write current completed stage to stage file
	expr $i + 1 > $STAGE_FILE
done

# clean up
rm $STAGE_FILE

infoMsg "************************************************************************"
infoMsg "*"
infoMsg "* Installation and compilation of MIRA is finished."
infoMsg "* Please add the following environment variables to your configuration:"
infoMsg "*    export MIRA_PATH=$INSTALL_DIR"
infoMsg "*    export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:$INSTALL_DIR/lib"
infoMsg "*    export PATH=\$PATH:$INSTALL_DIR/bin       (this is optional)"
infoMsg "*    source $INSTALL_DIR/scripts/mirabash"
infoMsg "* or use"
infoMsg "*    source $START_BASH"
infoMsg "* in your bash console to get started"
infoMsg "************************************************************************"
infoMsg ""