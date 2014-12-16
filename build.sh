#!/usr/bin/env bash

bundle exec berks vendor ops/packer/cookbooks
cp ops/packer/cudnn-6.5-linux-R1.tgz ops/packer/cookbooks/caffe/files/default/cudnn-tarball
packer build ops/packer/caffe.json
