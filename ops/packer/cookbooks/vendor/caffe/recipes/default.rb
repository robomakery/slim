

include_recipe "apt"

# common packages
%w{ git ruby tree mutt vim emacs yasnippet ldapscripts }.each do |p|
  package p
end
