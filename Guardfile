# A sample Guardfile
# More info at https://github.com/guard/guard#readme

## Uncomment and set this to only include directories you want to watch
# directories %w(app lib config test spec features)

## Uncomment to clear the screen before every task
# clearing :on

## Guard internally checks for changes in the Guardfile and exits.
## If you want Guard to automatically start up again, run guard in a
## shell loop, e.g.:
##
##  $ while bundle exec guard; do echo "Restarting Guard..."; done
##
## Note: if you are using the `directories` clause above and you are not
## watching the project directory ('.'), then you will want to move
## the Guardfile to a watched dir and symlink it back, e.g.
#
#  $ mkdir config
#  $ mv Guardfile config/
#  $ ln -s config/Guardfile .
#
# and, you'll have to watch "config/Guardfile" instead of "Guardfile"


guard :shell do
  watch %r{ros/src/slim_description/urdf/slim_no_base.xacro} do |m|
    n m[0], 'changed'
    file = Tempfile.new('urdf')
    begin
      slim_desc = `rospack find slim_description`.chomp
      slim_urdf = `rosrun xacro xacro.py #{slim_desc}/urdf/slim.urdf.xacro`
      file.write slim_urdf
      file.rewind
      `rosparam set /robot_description -t #{file.path}`
    ensure
      file.close
      file.unlink
    end
  end
end

  
