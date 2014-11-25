#!/usr/bin/env bash

bundle exec berks vendor ops/packer/cookbooks/vendor
packer build ops/packer/caffe.json

