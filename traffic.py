imports
import numpy as np
import matplotlib.pyplot as plt

def Traffic(n, vmax_lorries, vmax_cars, vmax_bikes, density, percent_lorries, percent_bikes, prob_car_daw,lane_change_prob,
             blockage, evasive, max_iterations, road_type, dis_type, output):
    ''' Function to generate a road and cars which move according to a set of rules.
        Inputs:
        n = length of the road.
        vmax_lorries = max speed of the lorries (if multiple lane system).
        vmax_cars = max speed of the cars.
        vmax_bikes = max speed of the motorbikes (if multiple lane system).
        percent_lorries = percentage of the vehicles that are lorries (if multiple lane system).
        percent_bikes = percentage of the vehicles that are motorbikes (if multiple lane system).
        density = density of cars on the road.
        prob_car_daw = probability of the vehicles dawdeling.
        lane_change_prob= probability of the vehicles changing lane if possible. 
        evasive = whether the overtaking rules dictate that whether cars should force overtaking for safety should take place,
            for evasice overtaking- "evasive", for standard rules, else.
        blockage = fraction of the road at which the lane closure occurs.
        max_iterations = number of timesteps.
        road_type= Whether we have a multiple or single laned road, for multiple- "Multi", for single, else.
        dis_type = Type of Visual representation of traffic- for speed of cars as different colours- "Speed", for vehicle type,
                   else
        display = Type of output, for the visual representation, "Matrix", else, just returns data array.
    ''' 
    #adding one to vmax magnitudes to account for element = speed + 1 
    vmax_lorries+=1
    vmax_cars+=1
    vmax_bikes+=1
    
    #determining whether a single lane or triple lane system.
    if road_type == 'Multi':
        no_lanes = 3
    else:
        no_lanes = 1
    #this had been hardcoded as it will not be varried.
    tol = 1e-4
    
    #Create empty road,  matrix to store iterations as well as datastorage arrays
    if abs(no_lanes -3 )<tol:
        tripleroad = np.zeros((no_lanes,n), dtype = complex)
        matrix = np.zeros(((no_lanes+1)*(max_iterations-1) -1 , n))
        tripledata= np.zeros((no_lanes*max_iterations, 12))
    else:
        road = np.zeros(n)
        matrix = np.zeros((max_iterations, n))
        data= np.zeros((max_iterations, 5))
         
    ### Car Generation ###
    if abs(no_lanes -3 )<tol:
        for j in range(no_lanes):
            for i in range(round(n*density)):
                ranspace = np.random.randint(0, n)
                while tripleroad[j,ranspace]!=0:
                    ranspace = np.random.randint(0, n)
                if np.random.randint(1, 100)<percent_lorries*100 and no_lanes - 1 <2:  
                    tripleroad[j, ranspace]=np.random.randint(2, vmax_lorries) -1j
                elif np.random.randint(1, 100*(1-percent_lorries))<percent_bikes*200:
                    tripleroad[j, ranspace]=np.random.randint(2, vmax_bikes) +1j
                else:
                    tripleroad[j, ranspace]=np.random.randint(2, vmax_cars)
    
    else:
        for i in range(round(n*density)):
            ranspace = np.random.randint(0, n)
            while road[ranspace]!=0:
                ranspace = np.random.randint(0, n)
            road[ranspace]=np.random.randint(2, vmax_cars)   
        
    #initlise timestep iterations 
    iterations = 0
   
    while iterations in range(max_iterations):
        
        #for triple road
        if abs(no_lanes -3 )<tol:

            #create loop like extention to the end of the road
            leftlane = np.concatenate((tripleroad[0,], tripleroad[0,]))
            middlelane = np.concatenate((tripleroad[1,], tripleroad[1,]))
            rightlane = np.concatenate((tripleroad[2,], tripleroad[2,]))

            for i in range(n-vmax_bikes, -1, -1):

                #setting the max speed depending on vehicle type
                if leftlane[i].imag == -1:
                    vmax = vmax_lorries
                elif leftlane[i].imag == 1:
                    vmax = vmax_bikes
                else:
                    vmax = vmax_cars

                #for the middle lane
                if middlelane[i].imag == -1:
                    vmax = vmax_lorries
                elif middlelane[i].imag == 1:
                    vmax = vmax_bikes
                else:
                    vmax = vmax_cars

                #for the right lane
                if rightlane[i].imag == -1:
                    vmax = vmax_lorries
                elif rightlane[i].imag == 1:
                    vmax = vmax_bikes
                else:
                    vmax = vmax_cars


                ### Overtaking ###

                #cars in the left lane:
                if leftlane[i].real>1 and abs(leftlane[i].imag-2)>tol :
                    #space in the middle lane for it to move forward into
                    if middlelane[i].real==0:
                        # are there cars in front?
                        if sum(leftlane[i+1:i+int(leftlane[i].real)].real)!=0:
                            #is it worth overtaking, the cars ahead:
                            count = 0    #Count is the distance between two cars.
                            while leftlane[i + count+1].real<1 and count<vmax:
                                count+=1
                            #if the velocity of the car is greater than the distance between
                            if  leftlane[i].real>count: 
                                #if the speed of the car is slowe than its max
                                if leftlane[i].real< vmax: #leftlane[i + count+1].real: 
                                    #ensure we stay within index- this will not matter when we move to periodic system
                                    if i-vmax>0:
                                        #ensuring the car behind in the middle lane wont have to slow down
                                        count2 = 0 
                                        while middlelane[i - count2 - 1].real<1 and count2<vmax:
                                            count2+=1
                                        #if the speed (1 timestep) of car behind is smaller than the distance
                                        if middlelane[[i - count2-1]].real<count2:

                                            #ensuring the car won't have to slow down in the middle lane
                                            count3 = 0 
                                            while middlelane[i + count3+1].real<1 and count3<=vmax:
                                                count3+=1

                                            #no cars infront at distance equivalent to that will be travelled in one time step.
                                            if  leftlane[i].real<count3:
                                                leftlane[i]+=1
                                                #move into the middle lane
                                                middlelane = np.delete(middlelane, i)
                                                middlelane = np.insert(middlelane, i, leftlane[i])

                                                leftlane = np.delete(leftlane, i)
                                                leftlane = np.insert(leftlane, i, 0)

                                                if middlelane[i]<vmax:
                                                    middlelane[i]+=1

                                            #if the cars that are in front are faster
                                            elif leftlane[i].real<middlelane[i + count3+1].real:
                                                #move into the middle lane
                                                middlelane = np.delete(middlelane, i)
                                                middlelane = np.insert(middlelane, i, leftlane[i])

                                                leftlane = np.delete(leftlane, i)
                                                leftlane = np.insert(leftlane, i, 0)

                                                if middlelane[i]<vmax:
                                                    middlelane[i]+=1
                                                    
                                                    
                                    ### evasive lane changing ###            
                                    #and if the car in question is traveling at at least half of its velocity.
                                    elif abs(leftlane[i + count].real - 1)<tol and leftlane[i].real -1 > vmax/2:
                                        if evasive == "evasive":
                                            #determining how far forward the car in the other lane is.
                                            count3 = 1 
                                            while middlelane[i + count3].real<1 and count3<=vmax:
                                                count3+=1
                                            # if the speed of the car in the other lane is too slow to ordinarily 
                                            # allow the car to overtake to safety
                                            if leftlane[i].real > middlelane[i + count3].real + count3:
                                                #reduce cars speed to be able to move into lane safely
                                                leftlane[i] = middlelane[i + count3].real + count3 -1 + comp

                                                #move into the other lane
                                                middlelane = np.delete(middlelane, i)
                                                middlelane = np.insert(middlelane, i, leftlane[i])

                                                leftlane = np.delete(leftlane, i)
                                                leftlane = np.insert(leftlane, i, 0)

                                            #the car in the other lane that is behind would ordinarily move past our car
                                            elif middlelane[i - count2].real - count2 > leftlane[i].real:
                                                #reduce other cars speed to allow the car to move into lane safely ahead of it
                                                comp2 = middlelane[i - count2].imag*1j
                                                middlelane[i - count2] = leftlane[i].real + count2 -1 +comp2

                                                #move into the other lane
                                                middlelane = np.delete(middlelane, i)
                                                middlelane = np.insert(middlelane, i, leftlane[i])

                                                leftlane = np.delete(leftlane, i)
                                                leftlane = np.insert(leftlane, i, 0)

                ### Moving to the slower lane ###

                #cars in the right lane:
                if rightlane[i].real>1 and abs(rightlane[i].imag-2)>tol:
                    #space in the middle lane for it to move forward into
                    if middlelane[i].real==0:

                        #if cars in the middle lane ahead are traveling at the same speed or faster
                        count4=0 
                        while middlelane[i + count4+1].real<1 and count4<=vmax:
                            count4+=1
                        #no cars in that space
                        if  rightlane[i].real<=count4:

                            #move into the middle lane
                            middlelane = np.delete(middlelane, i)
                            middlelane = np.insert(middlelane, i, rightlane[i])                                          

                            rightlane = np.delete(rightlane, i)
                            rightlane = np.insert(rightlane, i, 0)

                        #the car is traveling slower or equal than the cars in the space 
                        #and will have the distance between to not have to slow down
                        elif 2*rightlane[i].real<=middlelane[i + count4+1].real:

                            #move into the middle lane
                            middlelane = np.delete(middlelane, i)
                            middlelane = np.insert(middlelane, i, rightlane[i])                                          

                            rightlane = np.delete(rightlane, i)
                            rightlane = np.insert(rightlane, i, 0)


                ### cars in the middle lane can overtake or go into the slower lane
                ### Overtaking ###

                #cars in the middle lane:
                if middlelane[i].real>1 and abs(middlelane[i].imag-2)>tol:
                    #space in the right lane for it to move forward into
                    if rightlane[i].real==0:
                        # are there cars in front?
                        if sum(middlelane[i+1:i+int(middlelane[i].real)].real)!=0:
                            #is it worth overtaking, the cars ahead:
                            count = 0    #Count is the distance between two cars.
                            while middlelane[i + count+1].real<1 and count<vmax:
                                count+=1
                            #if the velocity of the car is greater than the distance between
                            if  middlelane[i].real>count: 
                                #if the speed of the car is slower than it's max
                                if middlelane[i].real< vmax: #middlelane[i + count+1].real: 
                                    #ensure we stay within index- this will not matter when we move to periodic system
                                    if i-vmax>0:
                                        #ensuring the car behind in the right lane wont have to slow down
                                        count2 = 0 
                                        while rightlane[i - count2 - 1].real<1 and count2<vmax:
                                            count2+=1
                                        #if the speed (1 timestep) of car behind is smaller than the distance
                                        if rightlane[[i - count2-1]].real<count2:
                                            #ensuring the car won't have to slow down in the right lane
                                            count3 = 0 
                                            while rightlane[i + count3+1].real<1 and count3<=vmax:
                                                count3+=1
                                            #no cars infront at distance equivalent to that will be travelled in one time step.
                                            if  middlelane[i].real<count3:
                                                #move into the right lane
                                                rightlane = np.delete(rightlane, i)
                                                rightlane = np.insert(rightlane, i, middlelane[i])

                                                middlelane = np.delete(middlelane, i)
                                                middlelane = np.insert(middlelane, i, 0)

                                                if rightlane[i]<vmax:
                                                    rightlane[i]+=1

                                            #if the cars that are in front are faster
                                            elif middlelane[i].real<rightlane[i + count3+1].real:

                                                #move into the right lane
                                                rightlane = np.delete(rightlane, i)
                                                rightlane = np.insert(rightlane, i, middlelane[i])

                                                middlelane = np.delete(middlelane, i)
                                                middleane = np.insert(middlelane, i, 0)

                                                if rightlane[i]<vmax:
                                                    rightlane[i]+=1

                ### Moving to the slower lane ###

                #cars in the middle lane:
                if middlelane[i].real>1 and abs(middlelane[i].imag-2)>tol:
                    #space in the left lane for it to move forward into
                    if leftlane[i].real==0:
                        #if cars in the left lane ahead are traveling at the same speed or faster
                        count4=0 
                        while leftlane[i + count4+1].real<1 and count4<=vmax:
                            count4+=1
                        #no cars in that space
                        if  middlelane[i].real<=count4:

                            #move into the other lane
                            leftlane = np.delete(leftlane, i)
                            leftlane = np.insert(leftlane, i, middlelane[i])                                          

                            middlelane = np.delete(middlelane, i)
                            middlelane = np.insert(middlelane, i, 0)


            #reducing each lane to its original size and back into the one road containing each lane.
            #It was important to double the size of the road so that the objects we are taking into consideration when 
            #checking cars forward and behind, dont exceed the limits of the array
            tripleroad[0,] = np.concatenate((leftlane[n:n+int(n/2)], leftlane[int(n/2):n]))
            tripleroad[1,] = np.concatenate((middlelane[n:n+int(n/2)], middlelane[int(n/2):n]))
            tripleroad[2,] = np.concatenate((rightlane[n:n+int(n/2)], rightlane[int(n/2):n]))

            for j in range(3):

                ##create loop like extention to the end of the road
                road = np.concatenate((tripleroad[j,], tripleroad[j,]))

                #iterate along the road
                for i in range(2*(n-vmax_bikes),  -1, -1):

                    #save the complex part to add on when any changes to the velocity are made
                    comp = road[i].imag*1j

                    #setting the max speed depending on vehicle type
                    if road[i].imag == -1:
                        vmax = vmax_lorries
                    elif road[i].imag == 1:
                        vmax = vmax_bikes
                    else:
                        vmax = vmax_cars

                    ### Rule 1 ###
                    # If the velocity v of the car is lower than vmax , and the distance to the next car or blockage
                    # ahead is larger than v + 1, the speed is increased by one.
                    if road[i].real>0 and abs(road[i].imag-2)>tol: 
                        #if there are no cars in front
                        if sum(road[i+1:i+int(road[i].real)].real)<tol and road[i].real<vmax:
                            road[i]+=1

                        ### Rule 2###

                        # If a driver at site i sees the next vehicle or blockage at site i+j, with j < v, 
                        # they reduce speed to j −1.  
                        else:
                            count5 = 1
                            #If there is a car directly in front slow to stationary.
                            while road[i + count5].real<tol and count5<vmax:
                                 #Count is the distance between two cars.
                                count5+=1
                            road[i]=count5 + comp
                ### Rule 4 ###

                # Each vehicle is advanced by v sites.
                # As the vechicles move forward we have to iterate in reverse as cars move forward.
                for i in range(2*n-vmax_bikes, -1, -1): 

                    #cars move forward 
                    if road[i].real>0 :                       
                        road= np.insert(road, i+int(road[i].real)-1, road[i])       
                        road = np.delete(road, i)


                #reduce the road back to the original shape
                tripleroad[j,] = np.concatenate((road[n:n+int(n/2)], road[int(n/2):n]))

                ###Rule 3 ###

                # The velocity of each moving vehicle is decreased by one with probability p.
                for i in range(n):
                    if np.random.randint(1, 100)<prob_car_daw*100:       
                        if tripleroad[j, i].real>2 and abs(road[i].imag-2)>tol:                                     
                            tripleroad[j, i]-=1

            #Visualisation
            if dis_type == "Speed":
                for j in range(3):
                    for i in range(n):
                        if tripleroad[j, i]==0:
                            matrix[4*(iterations-1) +j,i]= 15+3*j 
                        elif tripleroad[j, i].imag == 2:
                            matrix[4*(iterations-1) +j,i]= -50 #-30*j
                        else:
                            matrix[4*(iterations-1) +j,i]= 20+10*tripleroad[j, i].real
                        
            else:
                lorarr = np.zeros((3,n))
                cararr = np.zeros((3,n))
                bikearr = np.zeros((3,n))
                veharr =  np.zeros((3,n))
                
                for j in range(3):
                    for i in range(n):
                        if tripleroad[j, i]==0:
                            matrix[4*(iterations-1) +j,i]= 15+3*j
                            
                            
                        elif tripleroad[j, i].imag == -1:
                            matrix[4*(iterations-1) +j,i]= -30 #-30*j
                            lorarr[j,i]= tripleroad[j, i].real
                            veharr[j,i]= tripleroad[j, i].real
                            
                            
                        elif tripleroad[j, i].imag == 1:
                            matrix[4*(iterations-1) +j,i]= 50 #-30*j
                            bikearr[j,i]= tripleroad[j, i].real
                            veharr[j,i]= tripleroad[j, i].real
                            
                        elif tripleroad[j, i].imag == 2:
                            matrix[4*(iterations-1) +j,i]= -50 #-30*j
                            
                            
                        else:
                            matrix[4*(iterations-1) +j,i]= -10 #+30*j                                         
                            cararr[j,i]= tripleroad[j, i].real
                            veharr[j,i]= tripleroad[j, i].real
                                 
                lorratio = lorarr/vmax_lorries
                carratio = cararr/vmax_cars
                bikeratio = bikearr/vmax_bikes
            
                #data collection
                for j in range(3):
                    tripledata[3*iterations +j, 0]= iterations
                    #the next set must account for if there are no vehicles of that type in the lane
                    #due to division to find mean values
                    #number of lorries in each lane and the average speed for lorries in each lane
                    tripledata[3*iterations+j, 1]=np.count_nonzero(lorarr[j,:])
                    if np.count_nonzero(lorarr[j,:])>tol:
                        tripledata[3*iterations+j, 2]= (sum(lorarr[j,:]))/np.count_nonzero(lorarr[j,:])
                    else:
                        tripledata[3*iterations+j, 2]=0
                    tripledata[3*iterations+j, 3] = tripledata[3*iterations+j, 2]/vmax_lorries

                    #number of cars in each lane and the average speed for cars in each lane
                    tripledata[3*iterations+j, 4]=np.count_nonzero(cararr[j,:])
                    if np.count_nonzero(cararr[j,:])>tol:
                        tripledata[3*iterations+j, 5]= (sum(cararr[j,]))/np.count_nonzero(cararr[j,]) 
                    else:
                        tripledata[3*iterations+j, 5]=0
                    tripledata[3*iterations+j, 6] = tripledata[3*iterations+j, 5]/vmax_cars

                    #number of bikes in each lane and the average speed for bikes in each lane
                    tripledata[3*iterations+j, 7]=np.count_nonzero(bikearr[j,:])
                    if np.count_nonzero(bikearr[j,:])>tol:
                        tripledata[3*iterations+j, 8]= (sum(bikearr[j,]))/np.count_nonzero(bikearr[j,])     
                    else:
                        tripledata[3*iterations+j, 8]=0
                    tripledata[3*iterations+j, 9] = tripledata[3*iterations+j, 8]/vmax_bikes


                    #average speed of all vehicles in each lane
                    if np.count_nonzero(veharr[j,:])>tol:
                        tripledata[3*iterations+j, 10]= (sum(veharr[j,]))/np.count_nonzero(veharr[j,]) 
                    else:
                        tripledata[3*iterations+j, 10]= 0

                    #average speed to vmax ratio of all vehicles in each lane
                    if np.count_nonzero(veharr[j,:])>tol:
                        mean = ((sum(lorratio[j,:]))+ (sum(carratio[j,:]))+ (sum(bikeratio[j,:])))/np.count_nonzero(veharr[j,:])
                        tripledata[3*iterations+j, 11]= mean
                    else:
                        tripledata[3*iterations+j, 11]= 0
                    
                    
                    
            # for triple road - road blockage
            if iterations>max_iterations*blockage and (0.7*n+(iterations - max_iterations*blockage)*2)<=n:
                tripleroad[0, int(0.7*n): int(0.7*n+(iterations - max_iterations*blockage)*2)]= 1+2j 
                    
            ###next time step ###
            iterations +=1
         
        ###single laned road ### 
        else:
            #create loop like extention to the end of the road
            road = np.concatenate((road, road))

            #iterate along the road
            for i in range(2*(n-vmax_cars),  -1, -1):

                ### Rule 1 ###
                # If the velocity v of the car is lower than vmax , and the distance to the next car or blockage
                # ahead is larger than v + 1, the speed is increased by one.
                if road[i]>0: 
                    #if there are no cars in front
                    if sum(road[i+1:i+int(road[i])])<tol and road[i]<vmax_cars:
                        road[i]+=1

                    ### Rule 2###

                    # If a driver at site i sees the next vehicle or blockage at site i+j, with j < v, 
                    # they reduce speed to j −1.  
                    else:
                        count5 = 1
                        #If there is a car directly in front slow to stationary.
                        while road[i + count5]<tol and count5<vmax_cars:
                             #Count is the distance between two cars.
                            count5+=1
                        road[i]=count5

            ### Rule 4 ###

            # Each vehicle is advanced by v sites.
            # As the vechicles move forward we have to iterate in reverse as cars move forward.
            for i in range(2*n-vmax_cars, -1, -1): 

                #cars move forward 
                if road[i]>0 :                       
                    road= np.insert(road, i+int(road[i])-1, road[i])       
                    road = np.delete(road, i)

            #reduce the road back to the original shape
            road = np.concatenate((road[n:n+int(n/2)], road[int(n/2):n]))


            # The velocity of each moving vehicle is decreased by one with probability p.
            for i in range(n):
                if np.random.randint(1, 100)<prob_car_daw*100:       
                    if road[i]>1:                                     
                        road[i]-=1


            #Visualisation
            if dis_type == "Speed":
                for i in range(np.size(road)):
                    if road[i]==0:
                        matrix[iterations,i]= 0
                    else:
                        matrix[iterations,i]= 10*road[i]
            else:
                for i in range(np.size(road)):
                    if road[i]==0:
                        matrix[iterations,i]= 0
                    else:
                        matrix[iterations,i]= 1


            ### Data collection ###
            data[iterations, 0]= iterations
            #car density
            data[iterations, 1]= np.count_nonzero(road)/n
            #average speed- accouting for the fact the values are one greater than the speed
            data[iterations, 2]= sum(road)/np.count_nonzero(road) -1
            #average speed to vmax ratio
            data[iterations, 3]= (sum(road)/np.count_nonzero(road) -1)/(vmax_cars-1)
            #average speed over all iterations
            av = round(sum(data[:,2])/max_iterations, 2)

            ###next time step ###
            iterations +=1

    ### Display Road ###
    if output == "Matrix":
        fig11 = plt.figure(figsize=(40,60))
        ax11 = fig11.add_subplot(111)
        ax11.set_xlabel('Length Along Road [Cells]', fontsize=25)
        ax11.imshow (matrix) 
        if abs(no_lanes - 3) < tol:
            ax11.set_title("""Visual Repsresentation of Three Lane Traffic  With Density of {} vehicles/space Over {} 
            Time Steps With Dawdeling Probability of {}%.""".format(density, max_iterations, prob_car_daw), fontsize=25)
            
            ax11.set_ylabel('4 x Timestep [Arbitrary Units]', fontsize=25)
        else:
            ax11.set_title("""Visual Repsresentation of One Lane Traffic  With Density of {} vehicles/space Over {} 
            Time Steps With Dawdeling Probability of {}%. Average Velocity = {} 
            spaces/timestep.""".format(density, max_iterations, prob_car_daw, av), fontsize=25)
            
            ax11.set_ylabel('Timestep [Arbitrary Units]', fontsize=25)
    #return data to be plotted
    if abs(no_lanes - 3) < tol:
        return tripledata
    else:
        return data
