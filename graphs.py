from matplotlib import pyplot as plt

def setup_xyz_graph():
    plt.ion()
    fig=plt.figure()
    ax_x=plt.subplot(3,1,1)
    ax_y=plt.subplot(3,1,2)
    ax_z=plt.subplot(3,1,3)

    ax_x.title.set_text('X')
    ax_y.title.set_text('Y')
    ax_z.title.set_text('Z')
    
    ax_x.set_ylim([-45, 45])
    ax_y.set_ylim([-15, 50])
    ax_z.set_ylim([-25, 25])
    
    return fig,ax_x,ax_y,ax_z

def update_xyz_graph(ax_x,ax_y,ax_z,tvec):
    
    update_xyz_graph.x.append(tvec[0][0][0])
    update_xyz_graph.y.append(tvec[0][0][1])
    update_xyz_graph.z.append(tvec[0][0][2])

    ax_x.plot(update_xyz_graph.x)
    ax_y.plot(update_xyz_graph.y)
    ax_z.plot(update_xyz_graph.z)

    plt.draw()
    plt.pause(0.000001)


update_xyz_graph.x=[]
update_xyz_graph.y=[]
update_xyz_graph.z=[]