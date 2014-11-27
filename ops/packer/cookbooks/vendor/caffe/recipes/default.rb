
include_recipe "apt"
include_recipe "build-essential"
include_recipe "git"

# common packages
%w{ git ruby tree mutt vim emacs yasnippet ldapscripts }.each do |p|
  package p
end

# linux headers
package "linux-headers-#{node.os_version}"

# caffe dependencies
%w{ libprotobuf-dev libleveldb-dev libsnappy-dev libopencv-dev 
    libboost-all-dev libhdf5-serial-dev protobuf-compiler gcc-4.6 
    g++-4.6 gcc-4.6-multilib g++-4.6-multilib gfortran libjpeg62 
    libfreeimage-dev libatlas-base-dev git python-dev python-pip 
    libgflags-dev libgoogle-glog-dev liblmdb-dev }.each do |p|
  package p
end

software_dir = "/root/software"
directory software_dir
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

git "#{software_dir}/caffe" do
  repository "https://github.com/BVLC/caffe.git"
  revision "master"
  action :sync
end

cookbook_file "#{software_dir}/caffe/Makefile.config" do
  source "Makefile.config"
  mode 0644
end
