"""
Compute the different moments of inertia for the simple button
cf https://en.wikipedia.org/wiki/List_of_moments_of_inertia
"""
from __future__ import division, print_function

from lxml import etree

m_base = 5 # mass of the base in kg
m_cylinder = 5 # in kg
m_button = 0.1
outer_radius = 0.2 # in meter
inner_radius = 0.18  # in meter
h_base = 0.005 # height of the base in m
h_cylinder = 0.025
h_button = 0.03

def momentsThickWalledCylinder(m, r1, r2, h):
    """
    Thick-walled cylindrical tube with open ends,
    of inner radius r1, outer radius r2, length h and mass m.
    :return: [float] [Ixx, Iyy, Izz]
    """
    iz = (m / 2) * (r1**2 + r2**2)
    ix = (m / 12) * (3 * (r1**1 +  r2**2) + h**2)
    iy = ix
    return [ix, iy, iz]

def momentsCylinder(m, r, h):
    """
    Solid cylinder of radius r, height h and mass m.
    This is a special case of the thick-walled cylindrical tube (r1 = 0)
    :param m: (float) mass
    :param r: (float) radius
    :param h: (float) height
    :return: [float] [Ixx, Iyy, Izz]
    """
    return momentsThickWalledCylinder(m, 0, r, h)


I_base = momentsCylinder(m_base, outer_radius, h_base)
I_cylinder = momentsThickWalledCylinder(m_cylinder, inner_radius, outer_radius, h_cylinder)
I_button = momentsCylinder(m_button, inner_radius, h_button)

elements = zip(['base', 'cylinder', 'button'], [I_base, I_cylinder, I_button], [m_base, m_cylinder, m_button])

for name, moments, mass in elements:
    print("\n XML for {}".format(name))
    inertial = etree.Element('inertial')
    mass_node = etree.Element('mass')
    mass_node.text = str(mass)
    inertial.append(mass_node)
    inertia = etree.Element('inertia')

    for idx, e in enumerate(['ixx', 'iyy', 'izz']):
        elem = etree.Element(e)
        elem.text = str(moments[idx])
        inertia.append(elem)

    for e in ['ixy', 'ixz', 'iyz']:
        elem = etree.Element(e)
        elem.text = "0"
        inertia.append(elem)

    inertial.append(inertia)
    xml_string = etree.tostring(inertial, pretty_print=True)
    print(xml_string)
