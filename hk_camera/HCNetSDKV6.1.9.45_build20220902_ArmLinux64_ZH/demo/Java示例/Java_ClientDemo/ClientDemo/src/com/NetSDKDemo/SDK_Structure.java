package com.NetSDKDemo;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.List;
import com.sun.jna.Structure;

public class SDK_Structure extends Structure 
{	
      protected  List<String> getFieldOrder()
      {
    	  List<String> fieldOrderList = new ArrayList<String>();
    	  for (Class<?> cls = getClass();!cls.equals(SDK_Structure.class);cls = cls.getSuperclass()) 
    	  {
    		  Field[] fields = cls.getDeclaredFields();
    		  int modifiers;
    		  for (Field field : fields)
    		  {
    			  modifiers = field.getModifiers();
    			  if (Modifier.isStatic(modifiers) || !Modifier.isPublic(modifiers)) 
    			  {
    				  continue;
    			  }
    			  fieldOrderList.add(field.getName());
    		  }
    	}
    	//System.out.println(fieldOrderList);            
    	return fieldOrderList;
     }
}
