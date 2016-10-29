#!/bin/bash

thisFile=$_
if [ $BASH ] 
then
  # may be a relative or absolute path
  thisFile=${BASH_SOURCE[0]}
fi

set_path()
{
  export PATH=$PATH
  export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
  export CLASSPATH=$CLASSPATH:/usr/local/share/java/lcm.jar

  # enable some warnings by default
  export CXXFLAGS="$CXXFLAGS -Wreturn-type -Wuninitialized"
  export CFLAGS="$CFLAGS -Wreturn-type -Wuninitialized"
}

set_base()
{
  # use cd and pwd to get an absolute path
  configParentDir="$(cd "$(dirname "$thisFile")/../" && pwd)"

  # different cases for software/config or software/build/config
  case "$(basename $configParentDir)" in
    "software") export CODE_BASE=$(dirname $configParentDir);;
    *) echo "Warning: environment file is stored in unrecognized location: $thisFile";;
  esac
  export DATA_BASE=$CODE_BASE/../data
}


set_ros()
{
  if [ -f $CODE_BASE/catkin_ws/devel/setup.bash ]; then
    source $CODE_BASE/catkin_ws/devel/setup.bash
  else
    source /opt/ros/kinetic/setup.bash
  fi
}

# some useful commands
alias cdpokebot='cd $CODE_BASE'
alias cdpokebotdata='cd $POKEBOTDATA_BASE'
alias matlabdrake='cd $CODE_BASE/software; matlab -r "addpath_pods; addpath_drake"'
alias matlabpokebot='cd $CODE_BASE/software; matlab -nodesktop -nodisplay -nosplash -r "tic; addpath_pods; addpath_drake; toc; cd ../software/planning/ik_server/; ikTrajServerSocket;"'

alias gitsub='git submodule update --init --recursive'
alias gitpull='git -C $CODE_BASE pull'

alias rebash='source ~/.bashrc'
alias open='gnome-open'

alias yolo='rosservice call /robot1_SetSpeed 1600 180'
alias faster='rosservice call /robot1_SetSpeed 200 50'
alias fast='rosservice call /robot1_SetSpeed 100 30'
alias slow='rosservice call /robot1_SetSpeed 50 15'

alias gohome='rosservice call robot1_SetJoints "{j1: -0.32913211, j2: -39.41918751, j3: 33.58432661, j4: 5.55385908, j5: 6.08041137, j6: -5.98758923}"'
alias gohome2='rosservice call robot1_SetJoints "{j1: 0, j2: -40, j3: 33.58432661, j4: 5.55385908, j5: 6.08041137, j6: -5.98758923}"'
alias gozero='rosservice call robot1_SetJoints "{j1: 0, j2: 0, j3: 0, j4: 0, j5: 0, j6: 0}"'

alias teleop='rosrun teleop teleop'

alias pman='bot-procman-sheriff -l $CODE_BASE/software/config/procman.pmd'

alias roslocal='export ROS_MASTER_URI=http://localhost:11311'

alias catmake='cd $CODE_BASE/catkin_ws; catkin_make; cd -;'

alias pythonprof='python -m cProfile -o /tmp/tmp.cprof'
alias pythonprofthis='python -m cProfile -o '
alias showprof='pyprof2calltree -k -i /tmp/tmp.cprof'
alias showprofthis='pyprof2calltree -k -i '

alias showdev='for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev); do     (         syspath="${sysdevpath%/dev}";         devname="$(udevadm info -q name -p $syspath)";         [[ "$devname" == "bus/"* ]] && continue;         eval "$(udevadm info -q property --export -p $syspath)";         [[ -z "$ID_SERIAL" ]] && continue;         echo "/dev/$devname - $ID_SERIAL";     ); done'

ppms2mp4()
{
  bot-ppmsgz $1 mpeg4 10M 30 $1.mp4
}

function lowersuffix {
  cd "$1"
  find . -name '*.*' -exec sh -c '
  a=$(echo {} | sed -r "s/([^.]*)\$/\L\1/");
  [ "$a" != "{}" ] && mv "{}" "$a" ' \;
}

function ipmasq {
   if [ $# -eq 0 ]; then
     echo 'sharing wlan0 to eth0'
     sudo iptables -t nat -A POSTROUTING -o wlan0 -j MASQUERADE 
     sudo iptables -A FORWARD -i wlan0 -o eth0 -m state --state RELATED,ESTABLISHED -j ACCEPT 
     sudo iptables -A FORWARD -i eth0 -o wlan0 -j ACCEPT
   elif [ $# -eq 1 ]; then
     echo "sharing $1 to eth0"
     sudo iptables -t nat -A POSTROUTING -o $1 -j MASQUERADE
     sudo iptables -A FORWARD -i $1 -o eth0 -m state --state RELATED,ESTABLISHED -j ACCEPT
     sudo iptables -A FORWARD -i eth0 -o $1 -j ACCEPT
   elif [ $# -eq 2 ]; then
     echo "sharing $1 to $2"
     sudo iptables -t nat -A POSTROUTING -o $1 -j MASQUERADE
     sudo iptables -A FORWARD -i $1 -o $2 -m state --state RELATED,ESTABLISHED -j ACCEPT
     sudo iptables -A FORWARD -i $2 -o $1 -j ACCEPT
   fi
}

function set_bash {
   PROMPT_COMMAND='history -a'
   history -a

   # sorting in old style
   LC_COLLATE="C"
   export LC_COLLATE
   
   ulimit -c unlimited
   export HISTTIMEFORMAT="%d/%m/%y %T "
}

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64

set_base
set_ros
set_bash
set_path

exec "$@"  # very important for roslaunch remotely
