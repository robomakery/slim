
include_recipe "apt"
include_recipe "build-essential"
include_recipe "git"

# common packages
%w{ git ruby tree mutt vim emacs yasnippet ldapscripts }.each do |p|
  package p
end

