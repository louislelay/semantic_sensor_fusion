#!/bin/bash

# Build dockerfile
docker build --tag ros1_local .
docker image prune -f
