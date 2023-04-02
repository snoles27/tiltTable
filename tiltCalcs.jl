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

function ElAz2Normal(ElAz::Vector{Float64})

    #ElAz: [El, Az]

    return [cos(ElAz[1])*cos(ElAz[2]), cos(ElAz[1])*sin(ElAz[2]), sin(ElAz[1])]
end

function normal2PlanePoints(normal::Vector{Float64}; x0::Vector{Float64} = [1.3, 0.0, 1.3, 0.0])

    #state vector for this solve is x = [x1, z1, y2, z2]

    f13(x) = [-x[2] * x[3], -x[1] * x[4], x[1] * x[3]] / sqrt((x[1] * x[3])^2 + (x[1] * x[4])^2 + (x[2] * x[3])^2) - normal #normal vector equations
    f4(x) = sqrt(x[1]^2 + x[3]^2 + (x[2] - x[4])^2) - sqrt(2) * rodLoc #fixed point distance equation 
    f(x) = vcat(f13(x), f4(x))

    function fdot(x)
        x1 = x[1]
        z1 = x[2]
        y2 = x[3]
        z2 = x[4]

        df = zeros(4,4)
        df[1,1] = (y2*z1*(2*x1*y2^2 + 2*x1*z2^2))/(2*(x1^2*y2^2 + x1^2*z2^2 + y2^2*z1^2)^(3/2))
        df[1,2] = (y2^3*z1^2)/(x1^2*y2^2 + x1^2*z2^2 + y2^2*z1^2)^(3/2) - y2/(x1^2*y2^2 + x1^2*z2^2 + y2^2*z1^2)^(1/2)
        df[1,3] = (y2*z1*(2*y2*x1^2 + 2*y2*z1^2))/(2*(x1^2*y2^2 + x1^2*z2^2 + y2^2*z1^2)^(3/2)) - z1/(x1^2*y2^2 + x1^2*z2^2 + y2^2*z1^2)^(1/2)
        df[1,4] = (x1^2*y2*z1*z2)/(x1^2*y2^2 + x1^2*z2^2 + y2^2*z1^2)^(3/2)

        df[2,1] = (x1*z2*(2*x1*y2^2 + 2*x1*z2^2))/(2*(x1^2*y2^2 + x1^2*z2^2 + y2^2*z1^2)^(3/2)) - z2/(x1^2*y2^2 + x1^2*z2^2 + y2^2*z1^2)^(1/2)
        df[2,2] = (x1*y2^2*z1*z2)/(x1^2*y2^2 + x1^2*z2^2 + y2^2*z1^2)^(3/2)
        df[2,3] = (x1*z2*(2*y2*x1^2 + 2*y2*z1^2))/(2*(x1^2*y2^2 + x1^2*z2^2 + y2^2*z1^2)^(3/2))
        df[2,4] = (x1^3*z2^2)/(x1^2*y2^2 + x1^2*z2^2 + y2^2*z1^2)^(3/2) - x1/(x1^2*y2^2 + x1^2*z2^2 + y2^2*z1^2)^(1/2)

        df[3,1] = z2/(x1^2*y2^2 + x1^2*z2^2 + y2^2*z1^2)^(1/2) - (x1*z2*(2*x1*y2^2 + 2*x1*z2^2))/(2*(x1^2*y2^2 + x1^2*z2^2 + y2^2*z1^2)^(3/2))
        df[3,2] = -(x1*y2^2*z1*z2)/(x1^2*y2^2 + x1^2*z2^2 + y2^2*z1^2)^(3/2)
        df[3,3] = -(x1*z2*(2*y2*x1^2 + 2*y2*z1^2))/(2*(x1^2*y2^2 + x1^2*z2^2 + y2^2*z1^2)^(3/2))
        df[3,4] = x1/(x1^2*y2^2 + x1^2*z2^2 + y2^2*z1^2)^(1/2) - (x1^3*z2^2)/(x1^2*y2^2 + x1^2*z2^2 + y2^2*z1^2)^(3/2)

        df[4,:] = [x[1] x[2] (x[3] - x[4]) (x[4] - x[3])] / sqrt(x[1]^2 + x[2]^2 + (x[3] - x[4])^2)
        
        return df
    end
    
    return newtonraphson(f, fdot, x0)

end

let 

    normal2PlanePoints([1/sqrt(2), 0.0, 1/sqrt(2)])

    #thetas2ElAz([pi/4, 0.0])

    ##testing plane point finder
    # angles = range(-pi/2, pi/2, 100)
    # pygui(true)
    # for angle in angles
    #     point = findPlanePoint(angle)
    #     scatter(point[1], point[2])
    # end


end