
include_recipe "apt"
include_recipe "build-essential"
include_recipe "git"

software_dir = "/root/software"
directory software_dir

# common packages / config that I like - not specific to caffe
%w{ git ruby ruby-dev tree mutt vim emacs yasnippet ldapscripts }.each do |p|
  package p
end
git "#{software_dir}/emacs" do
  repository "https://github.com/dylanvaughn/emacs.git"
  revision "master"
  action :checkout
end
file "/root/.emacs" do
  owner "root"
  group "root"
  content '(setq shared-config-dir "~/software/emacs/") (load-file (concat shared-config-dir "dotemacs.el"))'
end

# start caffe install

# linux headers
package "linux-headers-#{node.os_version}"
# https://forums.aws.amazon.com/thread.jspa?messageID=558414
package "linux-image-generic"

# caffe dependencies
%w{ libprotobuf-dev libleveldb-dev libsnappy-dev libopencv-dev
    libboost-all-dev libhdf5-serial-dev protobuf-compiler gcc-4.6
    g++-4.6 gcc-4.6-multilib g++-4.6-multilib gfortran libjpeg62
    libfreeimage-dev libatlas-base-dev git python-dev python-pip
    libgflags-dev libgoogle-glog-dev liblmdb-dev }.each do |p|
  package p
end

# install cuda
remote_file "#{software_dir}/cuda-repo-ubuntu1404_6.5-14_amd64.deb" do
  source "http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1404/x86_64/cuda-repo-ubuntu1404_6.5-14_amd64.deb"
  action :create_if_missing
  notifies :run, 'bash[install-cuda-repo]', :immediately
end
bash 'install-cuda-repo' do
  action :nothing
  code "dpkg -i #{software_dir}/cuda-repo-ubuntu1404_6.5-14_amd64.deb && apt-get update"
end  
package 'cuda'

# install cudnn
cookbook_file "#{software_dir}/cudnn-6.5-linux-R1.tgz" do
  source "cudnn-6.5-linux-R1.tgz"
  mode 0644
end
execute 'tar -zxf cudnn-6.5-linux-R1.tgz' do
  cwd software_dir
  not_if { FileTest.exists? "#{software_dir}/cudnn-6.5-linux-R1" }
end
execute 'cp cudnn.h /usr/local/include' do
  cwd "#{software_dir}/cudnn-6.5-linux-R1"
  not_if { FileTest.exists? "/usr/local/include/cudnn.h" }
end
[ 'libcudnn_static.a', 'libcudnn.so.6.5.18' ].each do |lib|
  execute "cp #{lib} /usr/local/lib" do
    cwd "#{software_dir}/cudnn-6.5-linux-R1"
    not_if { FileTest.exists? "/usr/local/lib/#{lib}" }
  end
end
link "/usr/local/lib/libcudnn.so.6.5" do
  to "/usr/local/lib/libcudnn.so.6.5.18"
end
link "/usr/local/lib/libcudnn.so" do
  to "/usr/local/lib/libcudnn.so.6.5"
end

# set up LD_LIBRARY_PATH
file "/etc/ld.so.conf.d/caffe.conf" do
  owner "root"
  group "root"
  content "/usr/local/cuda-6.5/targets/x86_64-linux/lib"
  notifies :run, 'execute[ldconfig]', :immediately
end
execute 'ldconfig' do
  action :nothing
end

# download caffe and setup initial Makefile.config
git "#{software_dir}/caffe" do
  repository "https://github.com/BVLC/caffe.git"
  revision "c18d22eb92488f02c0256a3fe4ac20a8ad827596" # master as of Nov 25
  action :sync
end
cookbook_file "#{software_dir}/caffe/Makefile.config" do
  source "Makefile.config"
  mode 0644
end

# install python requirements
execute 'install-python-reqs' do
  cwd "#{software_dir}/caffe/python"
  command "(for req in $(cat requirements.txt); do pip install $req; done) && touch /root/.python-reqs-installed"
  creates "/root/.python-reqs-installed"
end

# make caffe!
execute 'build-caffe' do
  cwd "#{software_dir}/caffe"
  command "make all -j8"
  creates "#{software_dir}/caffe/build"
  notifies :run, 'execute[build-caffe-tests]', :immediately
end
execute 'build-caffe-tests' do
  cwd "#{software_dir}/caffe"
  command "make test -j8"
  action :nothing
end

# fix warning message 'libdc1394 error: Failed to initialize libdc1394' when running make runtest
# http://stackoverflow.com/a/26028597
link "/dev/raw1394" do
  to "/dev/null"
end
