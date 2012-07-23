#
# COPYRIGHT (C) 2005-2010
# RE2, INC.
# ALL RIGHTS RESERVED
#
# This is free software:
# you can redistribute it and/or modify it under the terms of the GNU General
# Public License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# You should have received a copy of the GNU General Public License along
# with this package.  If not, see <http://www.gnu.org/licenses/>.
#
# RE2, INC. disclaims all warranties with regard to this software, including
# all implied warranties of merchantability and fitness, in no event shall
# RE2, INC. be liable for any special, indirect or consequential damages or
# any damages whatsoever resulting from loss of use, data or profits, whether
# in an action of contract, negligence or other tortious action, arising out
# of or in connection with the use or performance of this software.
#
#
#

# This message is used to report the information about the Pan/Tilt unit
# that will be changing as the unit is used.

Header header

# This bool marks whether or not the unit is in Position Control.  If it
# is not, this means the unit is in Velocity Control.
bool currentlyInPositionControl

# These are the last recorded positions of the axes
float32 panPositionInRadians
float32 tiltPositionInRadians

# These are the last recorded velocities of the axes
float32 panVelocityInRadiansPerSec
float32 tiltVelocityInRadiansPerSec
