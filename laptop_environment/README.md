run time_sync.sh

if this fails

Connect via wifi or ethernet to a network with internet access!!!

manually do the following:

sudo /etc/init.d/chrony stop

sudo ntpdate 0.us.pool.ntp.org

sudo /etc/init.d/chrony start


