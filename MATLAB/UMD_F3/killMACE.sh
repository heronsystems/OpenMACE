#!/bin/bash

CLOSE_TERMINALS=0
while test $# -gt 0; do
        case "$1" in
                -x|--exit-terminals*)
                        shift
                        export CLOSE_TERMINALS=1
                        ;;
                *)
                        break
                        ;;
        esac
done

killall -9 roscore rosmaster rosout 
killall -9 MACE 
killall -9 yarn xterm 
killall -9 QtWebEngineProcess electron node
killall -9 arducopter mavproxy.py run_in_terminal python

if [ $CLOSE_TERMINALS -eq 1 ]  
  then
  echo "Killing all other open terminals..."
  # kill all terminals except the current one
  C=$(ps -p $(ps -p $$ -o ppid=) o args=)
  P1=$(ps -p $$ -o pid=)
  P2=$(ps -p $$ -o ppid=)
  echo $(pgrep $C)
  for p in $(pgrep $C | tail -n +2); do
      [ $p -ne $P1 ] && [ $p -ne $P2 ] && kill -9 $p
  done
fi

rm -f arduCopter.sh
