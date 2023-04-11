using PyPlot
using NLsolve
using Colors


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
    #normal: normal vector 

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

function normal2PlanePoints(normal::Vector{Float64}; x0::Vector{Float64} = [1.3, 0.2, 1.1, 0.1])

    #state vector for this solve is x = [x1, z1, y2, z2]
    #function is [normal vector matching equations, point distance fixed equations]
    ##DOESNT WORK - df Matrix singular on first iteratino

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
        
        display(df)

        return df
    end
    
    return newtonraphson(f, fdot, x0)

end

function ElAzGrid(theta1s::Vector{Float64}, theta2s::Vector{Float64})
    #theta1s: vector of first servo motor angles
    #theta2s: vector of second servo motor angles
    #returns El Matrix and Az matrix corresponding to full grid of all combos of theta1 and theta2
    n = length(theta1s)
    El = zeros(n, n)
    Az = zeros(n, n)

    for i = 1:n
        for j = 1:n
            ElAz = thetas2ElAz([theta1s[i], theta2s[j]])
            El[i,j] = ElAz[1]
            Az[i,j] = ElAz[2]
        end
    end

    return El, Az

end

function thetas2DelElAz(angles::Vector{Float64}, ElAz0::Vector{Float64})

    #angles: [theta1, theta2]
    #ElAz0: desired Elevation and Azimuth [El0, Az0]
    #returns: [delEl, delAz] where delAz is RoundNearest

    ElAz = thetas2ElAz(angles)
    delEl = ElAz[1] - ElAz0[1]
    delAz = rem2pi(ElAz[2] - ElAz0[2], RoundNearest)

    return [delEl, delAz]

end

function dWinding(point1::Vector{Float64}, point2::Vector{Float64}, func::Function)
    
    #point1: first point in servo angle space
    #point2: second point in servo angle space, 1-->2 is integratin direction
    #func: function that takes 2d inputs and has 2d outputs (thetas2delElAz for this problem, but only takes in points)

    out1 = func(point1)
    out2 = func(point2)

    a1 = atan(out1[2], out1[1])
    a2 = atan(out2[2], out2[1])

    return rem2pi(a2 - a1, RoundNearest)

end

function windingSegment(start::Vector{Float64}, stop::Vector{Float64}, func::Function; numStep::Int = 50)

    #start: 2x1 vector denoting start cooordinates of the segment we want to know the winding number of
    #stop: 2x1 vector denoting stop cooordinates of the segment we want to know the winding number of
    #func: function that takes 2d inputs and returns 2d outputs
    #numStep: steps to break segments into
    
    delta = stop - start
    step = delta./numStep

    winding = 0; #winding number sum for this segment 
    for i = 1:numStep
        a = start + step * (i-1)
        b = start + step * i
        winding = winding + dWinding(a, b, func)
    end
    return winding
end

function windingBox(verticies::Vector{Vector{Float64}}, func::Function)

    #verticies: nx(2x1) vector of vectors specifying verticies in the input space 
    #func: function that takes 2d inputs and has 2d outputs (thetas2delElAz for this problem, but only takes in points)
    #returns: winding number of all segments going around the verticies

    n = length(verticies)
    winding = 0
    for i = 1:n-1
        winding = winding + windingSegment(verticies[i], verticies[i+1], func)
    end
    #get the last leg going from last vertex to first vertex
    winding = winding + windingSegment(verticies[n], verticies[1], func)
    
    return winding

end

function colorPlot(ElAz0::Vector{Float64}; scale::Float64 = 0.5)
    #plots color grid in servo angle space
    #ElAz0: desired El and Az specifed with [El, Az] 2x1 vector

    #number of points in each servo range (total number is numPoints^2)
    numPoints = 50

    #set range of servo angles
    theta1 = collect(range(-pi/2, pi/2, numPoints))
    theta2 = collect(range(-pi/2, pi/2, numPoints))

    #get El and Az for every coordinate in servo angle space
    El, Az = ElAzGrid(theta1, theta2)

    #get difference from desired for every coordinate
    delEl = El .- ElAz0[1]
    delAz = rem2pi.(Az .- ElAz0[2], RoundNearest)

    pygui(true)
    for i = 1:numPoints
        for j = 1:numPoints
            rgb = getColor(delEl[i,j], delAz[i,j], scale = scale)
            scatter(theta1[i], theta2[j], color=rgb)
        end
    end
    xlabel("Servo 1 Angle (rad)")
    ylabel("Servo 2 Angle (rad)")

end

function getColor(delEl::Float64, delAz::Float64; scale::Float64 = 1.0)
    #returns RGB vector for a given delEl and delAz

    angle = atan(delAz, delEl) * 180/pi #returns angle from -pi to pi
    #if angle is negative, add 360
    if angle < 0.0
        angle = angle + 360
    end

    value = norm([delEl, delAz])/scale
    if value > 1
        value = 1
    end

    colorRGB = convert(RGB, HSV(angle, 1.0, value))
    
    return [Float64(red(colorRGB)), Float64(green(colorRGB)), Float64(blue(colorRGB))]


end


let 

    ElAz0 = thetas2ElAz([pi/3, 3pi/4])
    func(angles) = thetas2DelElAz(angles, ElAz0)
    verts = [[1.5,0], [1.5,1.], [0.5, 1.], [0.5, 0.]] #test box that encloses the root
    verts2 = [[1.5,-.75], [1.5,0.], [0.01, 0.], [0.01, -.75]]
    number = windingBox(verts2, func)
    
    #testing windingSegment
    
    # ElAz0 = thetas2ElAz([pi/3, 3pi/4])
    
    # func(angles) = thetas2DelElAz(angles, ElAz0)
    # start = [1.5, -1.5]
    # stop = [1.5, 1.]

    # winding = windingSegment(start, stop, func)


    #colorPlot(ElAz0, scale = 0.1)

    # del = thetas2DelElAz([pi/3, pi/4], [pi/3, 3pi/4])
    # display(del)

    # colorPlot([pi/3, 3pi/4], scale = 0.1)

    # #testing color generator by looking at the output space coloring
    # delAz = collect(range(-1,1,50))
    # delEl = collect(range(-1,1,50))

    # pygui(true)
    # for i = 1:50
    #     for j = 1:50
    #         rgb = getColor(delEl[i],delAz[j],scale = 0.3)
    #         scatter(delEl[i], delAz[j], color = rgb)
    #     end
    # end



    #some plotting testing
    # theta1 = collect(range(-pi/2, pi/2, 50))
    # theta2 = collect(range(-pi/2, pi/2, 50))

    # angleWant = pi/4

    # El, Az = ElAzGrid(theta1, theta2)

    # T2 = theta2' .* ones(length(theta1))
    # T1 = ones(length(theta1))' .* theta1

    # pygui(true)
    # surf(T1, T2, El, cmap=ColorMap("coolwarm"))
    # surf(T1, T2, angleWant * ones(length(theta1), length(theta2)))
    # xlabel("Servo 1 Angle (rad)")
    # ylabel("Servo 2 Angle (rad)")
    
    #thetas2ElAz([pi/4, 0.0])

    ##testing plane point finder
    # angles = range(-pi/2, pi/2, 100)
    # pygui(true)
    # for angle in angles
    #     point = findPlanePoint(angle)
    #     scatter(point[1], point[2])
    # end


end