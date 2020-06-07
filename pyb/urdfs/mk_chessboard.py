#!/usr/bin/env python3

class URDFPrinter():
  header_template = """
<?xml version="1.0" ?>

<robot name="%(name)s">

  <!-- Colors -->
  <material name="Black"> <color rgba="0 0 0 1.0"/> </material>
  <material name="White"> <color rgba="1 1 1 1.0"/> </material>
"""

  board_template = """
  <link name="chessboard">
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual> <origin rpy="0 0 0" xyz="%(xyz)s"/>
      <geometry> <box size="%(size)s" /> </geometry>
      <material name="White"/>
    </visual>
  </link>
  """

  square_template = """
  <link name="square-%(x)s-%(y)s">
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/> <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual> <origin rpy="0 0 0" xyz="%(xyz)s"/>
      <geometry> <box size="%(size)s" /> </geometry>
      <material name="Black"/>
    </visual>
  </link>

  <joint name="joint-s-%(x)s-%(y)s" type="fixed">
    <parent link="chessboard"/>
    <child link="square-%(x)s-%(y)s"/>
  </joint>
"""

  circle_template = """
  <link name="circle-%(x)s-%(y)s">
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/> <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual> <origin rpy="0 0 0" xyz="%(xyz)s"/>
      <geometry> <cylinder radius="%(radius)f" length="%(length)f"/> </geometry>
      <material name="Black"/>
    </visual>
  </link>

  <joint name="joint-c-%(x)s-%(y)s" type="fixed">
    <parent link="chessboard"/>
    <child link="circle-%(x)s-%(y)s"/>
  </joint>
"""

  footer_template = """
</robot>
"""

  base_name = "Chessboard"
  D = 0.0423 # TV
  X0 = -7*D
  Y0 = -4*D
  thickness = 0.01

  def print_header(self, f, name):
    print(self.header_template % {"name": name}, file=f)

  def print_footer(self, f):
    print(self.footer_template, file=f)

  def print_board(self, f, W, H):
    x = self.X0 + W * self.D / 2
    y = self.Y0 + H * self.D / 2
    w = (W+4) * self.D
    h = (H+4) * self.D
    print(self.board_template % {
      "xyz": ("%f %f %f" % (x, y, -self.thickness/2)),
      "size": ("%f %f %f" % (w, h, self.thickness))
      }, file=f)

  def print_tile(self, f, X, Y):
    x = self.X0 + (X + 0.5) * self.D
    y = self.Y0 + (Y + 0.5) * self.D
    print(self.square_template % {
      "x": X, "y": Y,
      "xyz": ("%f %f 0" % (x, y)),
      "size": ("%f %f %f" % (self.D, self.D, self.thickness)),
      }, file=f)

  def print_edge_tile(self, f, cx, cy, cr, sx, sy, sw, sh):
    print(self.square_template % {
      "x": sx, "y": sy,
      "xyz": ("%f %f 0" % (sx, sy)),
      "size": ("%f %f %f" % (sw, sh, self.thickness)),
      }, file=f)

    print(self.circle_template % {
      "x": cx, "y": cy,
      "xyz": ("%f %f 0" % (cx, cy)),
      "radius": (cr),
      "length": (self.thickness),
      }, file=f)

  def print_bottom_left_tile(self, f):
    cx, cy, cr = self.X0-self.D/2, self.Y0-self.D/2, self.D/2
    sx, sy, sd = self.X0-self.D/4, self.Y0-self.D/4, self.D/2
    self.print_edge_tile(f, cx, cy, cr, sx, sy, sd, sd)

  def print_bottom_right_tile(self, f, W):
    cx, cy, cr = self.X0+(W+0.5)*self.D, self.Y0-self.D/2, self.D/2
    sx, sy, sd = self.X0+(W+0.25)*self.D, self.Y0-self.D/4, self.D/2
    self.print_edge_tile(f, cx, cy, cr, sx, sy, sd, sd)

  def print_bottom_tile(self, f, X):
    cx, cy, cr = self.X0+(X+0.5)*self.D, self.Y0-self.D/2, self.D/2
    sx, sy, sw, sh = self.X0+(X+0.5)*self.D, self.Y0-self.D/4, self.D, self.D/2
    self.print_edge_tile(f, cx, cy, cr, sx, sy, sw, sh)

  def print_top_tile(self, f, X, H):
    cx, cy, cr = self.X0+(X+0.5)*self.D, self.Y0+(H+0.5)*self.D, self.D/2
    sx, sy, sw, sh = self.X0+(X+0.5)*self.D, self.Y0+(H+0.25)*self.D, self.D, self.D/2
    self.print_edge_tile(f, cx, cy, cr, sx, sy, sw, sh)

  def print_left_tile(self, f, Y):
    cx, cy, cr = self.X0-self.D/2, self.Y0+(Y+0.5)*self.D, self.D/2
    sx, sy, sw, sh = self.X0-self.D/4, self.Y0+(Y+0.5)*self.D, self.D/2, self.D
    self.print_edge_tile(f, cx, cy, cr, sx, sy, sw, sh)

  def print_right_tile(self, f, Y, W):
    cx, cy, cr = self.X0+(W+0.5)*self.D, self.Y0+(Y+0.5)*self.D, self.D/2
    sx, sy, sw, sh = self.X0+(W+0.25)*self.D, self.Y0+(Y+0.5)*self.D, self.D/2, self.D
    self.print_edge_tile(f, cx, cy, cr, sx, sy, sw, sh)

  def print(self, f, W, H):
    self.print_header(f, "Chessboard")
    self.print_board(f, W, H)
    for X in range(W):
      for Y in range(H):
        if (X + Y) % 2 == 0:
          self.print_tile(f, X, Y)

    self.print_bottom_left_tile(f)
    self.print_bottom_right_tile(f, W)

    for X in range(1, W, 2):
      self.print_bottom_tile(f, X)

    for X in range(0, W, 2):
      self.print_top_tile(f, X, H)

    for Y in range(1, H, 2):
      self.print_left_tile(f, Y)

    for Y in range(1, H, 2):
      self.print_right_tile(f, Y, W)

    self.print_footer(f)

p = URDFPrinter()
p.print(open("chessboard.urdf", "w"), 13, 8)
