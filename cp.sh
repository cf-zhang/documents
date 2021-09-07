#!/usr/bin/expect -f
set file [lindex $argv 0]
set passwd ld
spawn scp $file ld@10.3.0.3:/home/ld/
expect "ld@10.3.0.3's password:"
set timeout 30
send "$passwd\r"
send "exit\r"
expect eof

spawn scp $file ld@10.3.0.12:/home/ld/
expect "ld@10.3.0.12's password:"
set timeout 30
send "$passwd\r"
send "exit\r"
expect eof


spawn scp $file ld@10.3.0.5:/home/ld/
expect "ld@10.3.0.5's password:"
set timeout 30
send "$passwd\r"
send "exit\r"
expect eof

spawn scp $file ld@10.3.0.18:/home/ld/
expect "ld@10.3.0.18's password:"
set timeout 30
send "$passwd\r"
send "exit\r"
expect eof

spawn scp $file ld@10.3.0.16:/home/ld/
expect "ld@10.3.0.16's password:"
set timeout 30
send "$passwd\r"
send "exit\r"
expect eof

spawn scp $file ld@10.3.0.17:/home/ld/
expect "ld@10.3.0.17's password:"
set timeout 30
send "$passwd\r"
send "exit\r"
expect eof
