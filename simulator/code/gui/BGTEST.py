from tkinter import *
import sys, os
import shutil


def mainstartbutton():  # Main start Button, kills window and continues simulator
    top.destroy()


def savebtnCC():     # Save button for changing current configuration
    if (entryDays.get()) > 7:
        entryDays.set("Max 7 days")
    else:
        numUser.set(entryUser.get())
        numDays.set(entryDays.get())
        entryUser.set("")
        entryDays.set("")


def savebtnNL():   # Save button for New List Event
    cityName.set(entryCity.get())
    setchoice.set(1)
    entryCity.set("")


def saverunNL():  # Save and Run button for New List Event
    cityName.set(entryCity.get())
    entryCity.set("")
    setchoice.set(1)
    top.destroy()


def dispListContent(el):  # Gets current selection from listbox and writes to info label under box
    global labelListboxNumRun
    labelListboxNumRun = StringVar()
    global labelListboxDaysRun
    labelListboxDaysRun = StringVar()
    list = listfromSaved
    selection = slistbox.get(slistbox.curselection())
    for element in list:
        if element['id'] == selection[0]:
            labelListboxName.set(element['name'])
            labelListboxNum.set(["Users:", element['us']])
            labelListboxDays.set(["Day(s):", element['days']])
            labelListboxNumRun.set(element['us'])
            labelListboxDaysRun.set(element['days'])



def listboxRunbtn():
    global selection
    list = listfromSaved
    ''.join((labelListboxName.get()))
    ''.join((labelListboxNumRun.get()))
    ''.join((labelListboxDaysRun.get()))
    cityName.set("no")
    numUser.set((labelListboxNumRun.get()))
    numDays.set((labelListboxDaysRun.get()))
    setchoice.set(1)
    selection = slistbox.get(slistbox.curselection())
    
    top.destroy()






saved=[]
with open('../data/Inputs/SavedList.txt', 'r') as data:
    for line in data:
        
        listsaved={}
        p=line.split()
        listsaved['id']=int(p[0])
        listsaved['name']=p[1]
        listsaved['us']=int(p[2])
        listsaved['days']=int(p[3])
        saved.append(listsaved)
        
with open('../data/Setup.txt', 'r') as data:
    count=0
    for line in data:
        if count==1:
            p=line.split()
            days=int(p[4])


        if count==4:
            r=line.split()
            num_user=int(r[4])
                   
        count+=1

global top
top = Tk()


global setchoice
setchoice = IntVar()
setchoice.set(0)

global entryUser
entryUser = IntVar()
entryUser.set("")

global numUser
numUser = IntVar()
numUser.set(num_user)

global entryDays
entryDays = IntVar()
entryDays.set("")

global numDays
numDays = IntVar()
numDays.set(days)

global entryCity
entryCity = StringVar()

global cityName
cityName = StringVar()

global listfromSaved
listfromSaved = saved

global selection
global labelListboxName
global labelListboxNum
global labelListboxDays
labelListboxName = StringVar()
labelListboxNum = IntVar()
labelListboxDays = IntVar()

top.title("CrowdSenSim V.1.1.0")

C = Canvas(top, bg="blue", height=767, width=1150)
filename = PhotoImage(file="CrowdSenSimMainBG2.gif")
background_label = Label(top, image=filename)
background_label.place(x=0, y=0, relwidth=1, relheight=1)
C.pack()

# Start button
img2 = PhotoImage(file="CSSWelcomPWRBTN.gif")
powerbtn = Button(top, image=img2, command=saverunNL)
powerbtn.place(x=870, y=580)


# Labels for Current Config
cclab1 = Label(top, textvariable=numUser, font=("Helvetica", 16), background="white")
cclab1.place(x=441, y=237, height=41, width=251)
cclab1.config(relief=SUNKEN)
cclab2 = Label(top, textvariable=numDays, font=("Helvetica", 16), background="white")
cclab2.place(x=441, y=325, height=41, width=251)
cclab2.config(relief=SUNKEN)


# Entry boxes and save button for changing config
ccentry1 = Entry(top, textvariable=entryUser, font=("Helvetica", 16), background="white")
ccentry1.place(x=815, y=237, height=41, width=251)
ccentry1.config(justify=CENTER)
ccentry2 = Entry(top, textvariable=entryDays, font=("Helvetica", 16), background="white")
ccentry2.place(x=815, y=325, height=41, width=251)
ccentry2.config(justify=CENTER)
ccsavebtn = Button(top, text="SAVE", font=("Helvetica", 15), bg="#159cd1", command=savebtnCC)
ccsavebtn.place(x=887, y=381, height=28, width=102)


# Entrybox and buttons for Create new List
clentry1 = Entry(top, textvariable=entryCity, font=("Helvetica", 16), background="white")
clentry1.place(x=61, y=659, height=41, width=251)
clsavebtn = Button(top, text="SAVE", font=("Helvetica", 15), bg="#159cd1", command=savebtnNL)
clsavebtn.place(x=210, y=714, height=28, width=102)
clsaverunbtn = Button(top, text="SAVE & RUN", font=("Helvetica", 15), bg="#159cd1", command=saverunNL)
clsaverunbtn.place(x=59, y=714, height=28, width=140)


# Scrolled listbox, buttons and labels for Using saved list and manage lists
global slistbox
slistbox = Listbox(top, font=("Helvetica", 16), background="white", selectmode=BROWSE)
for item in saved:
    slistbox.insert(END,[item['id'],item['name']])
slistbox.bind("<<ListboxSelect>>", dispListContent)
scrollbar = Scrollbar(slistbox, orient=VERTICAL)
slistbox.config(yscrollcommand=scrollbar.set)
scrollbar.config(command=slistbox.yview)
slistbox.place(x=426, y=534, height=109, width=165)

scrlrunbtn01 = Button(top, text="RUN", font=("Helvetica", 15), bg="#159cd1", command=listboxRunbtn)
scrlrunbtn01.place(x=600, y=533, height=28, width=102)
scrldelbtn = Button(top, text="DEL", font=("Helvetica", 15), bg="#159cd1",
                    command=lambda slistbox=slistbox: slistbox.delete(ANCHOR))
scrldelbtn.place(x=600, y=576, height=28, width=102)

scrllab1 = Label(top, textvariable=labelListboxName, font=("Helvetica", 16), background="white")
scrllab1.place(x=426, y=648, height=32, width=251)
scrllab1.config(relief=SUNKEN)
scrllab2 = Label(top, textvariable=labelListboxNum, font=("Helvetica", 16), background="white")
scrllab2.place(x=426, y=683, height=32, width=251)
scrllab2.config(relief=SUNKEN)
scrllab3 = Label(top, textvariable=labelListboxDays, font=("Helvetica", 16), background="white")
scrllab3.place(x=426, y=718, height=32, width=251)
scrllab3.config(relief=SUNKEN)

top.resizable(False, False)
top.mainloop()

if (cityName.get()=='no' or cityName.get()==''):
    print ("yes",selection[0],"no",numUser.get(), numDays.get(),"ddf")
    
else:
    stringname=cityName.get()
    stringname=stringname.replace(" ", "-")
    print ("no",0,stringname,numUser.get(), numDays.get(),"ddf")


