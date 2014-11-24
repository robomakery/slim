# -*- mode: ruby -*-
# vi: set ft=ruby :

VAGRANTFILE_API_VERSION = "2"

Vagrant.configure(VAGRANTFILE_API_VERSION) do |config|
  config.vm.box = "janihur/ubuntu-1404-desktop"
  config.vm.provision :shell, path: "ops/vagrant/bootstrap.sh"
  config.vm.provider "virtualbox" do |v|
    v.gui = true
  end
end
