#!/bin/bash

echo "Copying logs..."
scp admin@10.36.48.2:/U/logs/* ./logs
echo "Deleting old logs..."
ssh admin@10.36.48.2 "rm -f /U/logs/*"