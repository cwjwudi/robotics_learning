print("Hello")

a = 1
b = 2
c = a+b
print(c)

list = [1,"Hello",23,"test"]
print(list)
print(list[0])
print(type(list))

# if-else
c = 10
if c>0 and c<5:
    print('c is betweeen 0 and 5')
elif c>6 and c < 9:
    print('c is between 6 and 9')
else:
    print('c is greater than 10')

i = 1
while i<10:
    print(i)
    if (i == 6):
        break;
    # i = i+1
    i +=1;

for x in range(5):
    print(x)

def my_function():
    print("THis is my function")

my_function()

def my_add(a,b):
    return a+b

sum = my_add(3,4)
print(sum)
