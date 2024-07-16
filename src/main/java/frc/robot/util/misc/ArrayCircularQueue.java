package frc.robot.util.misc;

import java.util.ArrayList;
import java.util.EmptyStackException;

//copied from here bc I'm lazy and dont like writing data structures: https://www.geeksforgeeks.org/introduction-to-circular-queue/
public class ArrayCircularQueue<E> {
    // Declaring the class variables.
    private int size, front, rear;
    
    // Declaring array list of integer type.
    private ArrayList<E> queue = new ArrayList<>();
    
    // Constructor
    public ArrayCircularQueue(int size, E defaultValue)
    {
        this.size = size;
        this.front = this.rear = -1;

        for (int i = 0; i < size; i++) {
            queue.add(i, defaultValue);
        }
    }
    
    // Method to insert a new element in the queue.
    public void enQueue(E data)
    {
        
        // Condition if queue is full.
        if((front == 0 && rear == size - 1) ||
        (rear == (front - 1) % (size - 1)))
        {
            System.out.print("Queue is Full");
        }
    
        // condition for empty queue.
        else if(front == -1)
        {
            front = 0;
            rear = 0;
            queue.add(rear, data);
        }
    
        else if(rear == size - 1 && front != 0)
        {
            rear = 0;
            queue.set(rear, data);
        }
    
        else
        {
            rear = (rear + 1);
        
            // Adding a new element if 
            if(front <= rear)
            {
                queue.add(rear, data);
            }
        
            // Else updating old value
            else
            {
                queue.set(rear, data);
            }
        }
    }
    
    // Function to dequeue an element
    // form th queue.
    public E deQueue()
    {
        E temp;
    
        // Condition for empty queue.
        if(front == -1)
        {
            System.out.print("Queue is Empty");
            
            // Return -1 in case of empty queue
            //I dont think this is the right exception but it should work
            throw new EmptyStackException(); 
        }
    
        temp = queue.get(front);
    
        // Condition for only one element
        if(front == rear)
        {
            front = -1;
            rear = -1;
        }
    
        else if(front == size - 1)
        {
            front = 0;
        }
        else
        {
            front = front + 1;
        }
        
        // Returns the dequeued element
        return temp;
    }

    public E get(int index) {
        return queue.get(index);
    }
    
    
}