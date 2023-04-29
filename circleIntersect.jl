using LinearAlgebra

function circle(point::Vector{Float64}, center::Vector{Float64}, r::Float64)
    #function for n dimensional circle

    #inputs
    #point: nx1 vector for test point
    #cetner: nx1 vector for circle center
    #r: radius of circle

    sum = 0
    for i = 1:length(point)
        sum = sum + (point[i] - center[i])^2
    end
    return sum - r^2
end

function circleDot(point::Vector{Float64}, center::Vector{Float64}, r::Float64)
    #derivative function for n dimensional circle

    #inputs
    #point: nx1 vector for test point
    #cetner: nx1 vector for circle center
    #r: radius of circle

    m = Array{Float64}(undef, 1, 0)
    for i = 1:length(point)
        m = hcat(m, 2 * (point[i] - center[i]))
    end
    return m

end

function circleCross(point::Vector{Float64}, center1::Vector{Float64}, center2::Vector{Float64}, r1::Float64, r2::Float64)
    #point: test location 
    #center1: Vector in R2 specifying center of circle 1 (ex: [x,y])
    #center2: Vector in R2 specifying center of circle 2 (ex: [x,y])
    #r1: radius of circle 1
    #r2: radius of circle 2
    #returns f(point). Fucntion that will be 0,0 when the entered x is a point of intersection

    return [circle(point, center1, r1); circle(point, center2, r2)]

end

function circleCrossDot(point::Vector{Float64}, center1::Vector{Float64}, center2::Vector{Float64}, r1::Float64, r2::Float64)
    #point: test x location 
    #center1: Vector in R2 specifying center of circle 1 (ex: [x,y])
    #center2: Vector in R2 specifying center of circle 2 (ex: [x,y])
    #r1: radius of circle 1
    #r2: radius of circle 2
    #returns df/dx. derivative of circleCross wrt point(nxn)

    return [circleDot(point, center1, r1); circleDot(point, center2, r2)]
    
end

function newtonraphson(f::Function, fdot::Function, x0::Vector{Float64}; tol::Float64 = 1e-9)
    #vector newton raphson solve

    #f: function searching for f(x) = 0
    #fdot: derivative of f 
    #x0: initial guess
    #tol: solution tolerance

    #####
    countLim = 30
    #####
    
    resid = ones(length(x0)) * Inf
    xi = x0
    count = 0
    while (norm(resid) > tol)
        xi1 = nrStep(xi, f, fdot)
        resid = xi1 - xi
        xi = xi1
        count = count + 1
        if(count > countLim)
            error("newtonraphson reached step limit -- check inputs")
        end
    end

    return xi
    
end

function newtonraphson(f::Function, fdot::Function, x0::Float64; tol::Float64 = 1e-9)
    #1D newton raphson: UNTESTED

    #####
    countLim = 30
    #####
    
    resid = Inf
    xi = x0
    count = 0
    while (resid > tol)
        xi1 = nrStep(xi, f, fdot)
        resid = xi1 - xi
        xi = xi1
        count = count + 1
        if(count > countLim)
            error("newtonraphson reached step limit -- check inputs")
        end
    end
    return xi
end

function nrStep(x::Vector{Float64}, f::Function, fdot::Function)
    #vector newton raphson step
    return x - fdot(x)\f(x)
end

function nrStep(x::Float64, f::Function, fdot::Function)
    #1D newton raphson step. UNTESTED
    return x - f(x)/fdot(x)
end

function findCircleIntersect(center1::Vector{Float64}, center2::Vector{Float64}, r1::Float64, r2::Float64, x0::Vector{Float64}; tol::Float64 = 1e-9)
    

    crossFunc(x) = circleCross(x, center1, center2, r1, r2)
    crossFuncDot(x) = circleCrossDot(x, center1, center2, r1, r2)

    return newtonraphson(crossFunc, crossFuncDot, x0, tol = tol)

end

# let 

#     ##testing circleIntersection
#     center1 = [0., 0.]
#     center2 = [1.33, -1.88]
#     r1 = 1.3
#     r2 = 2.75
#     testPoint = [1.30, 0.0]
#     f(x) = circleCross(x, center2, center1, r2, r1)
#     fdot(x) = circleCrossDot(x, center2, center1, r2, r1)

#     display(nrStep(testPoint, f, fdot));
#     display(f(testPoint))
#     display(fdot(testPoint))
#     display(newtonraphson(f, fdot, testPoint))


#     # ans = newtonraphson(f, fdot, testPoint)

#     #findCircleIntersect(center1, center2, r1, r2, testPoint)

#  end