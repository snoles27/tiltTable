using PyPlot
using NLsolve


include("circleIntersect.jl")


######DESIGN PARAMETERS########
rServ = 0.9  #radius of the servo arm (inches)
rodLoc = 1.3 #location of rod attachement to plate, distance to center (inches)
lRod = 3.0 #length of actuation rod, servo attachment --> ball center (inches) 
###############################

##useful nums derived from PARAMETERS##
zserv = -sqrt(lRod^2 - (rServ/2)^2) #z poition of servo center
xyserv = rodLoc - rServ/2 #x or y position of servoCenter
#######################################


function rodEndLoc(theta::Float64)

    #returns: (x[y], z) position of servo end of rod

    return [xyserv + rServ * cos(theta), zserv + rServ * sin(theta)]

end

function findPlanePoint(theta)

    rodEnd = rodEndLoc(theta) #center of circle defined by rod degree of freedom

    return findCircleIntersect(rodEnd, [0.0,0.0], lRod, rodLoc, [rodLoc, 0.0])

end

function getTableNormal(theta1, theta2)

    xz = findPlanePoint(theta1)
    yz = findPlanePoint(theta2)

    p1 = [xz[1], 0, xz[2]]
    p2 = [0, yz[1], yz[2]]

    n = cross(p1, p2)

    return n/norm(n)

end

function getElAz(normal::Vector{Float64})

    El = asin(normal[3])
    Az = atan(normal[2], normal[1])

    return [El, Az]

end

function thetas2ElAz(angles::Vector{Float64})

    nhat = getTableNormal(angles[1], angles[2])

    return getElAz(nhat)
end

function ElAz2Thetas(ElAz::Vector{Float64})

    thetas0 = [0.1,0.2]

    function f!(F, angles)
        F = thetas2ElAz(angles) - ElAz
    end

    return nlsolve(f!, thetas0)
end

function f!(F, angles)

    println(angles)
    ElAz = [1.04, 3.0]
    current = thetas2ElAz(angles)
    display(current)
    F = current - ElAz

end

let 
    

    #thetas2ElAz([pi/4, 0.0])

    ##testing plane point finder
    # angles = range(-pi/2, pi/2, 100)
    # pygui(true)
    # for angle in angles
    #     point = findPlanePoint(angle)
    #     scatter(point[1], point[2])
    # end


end