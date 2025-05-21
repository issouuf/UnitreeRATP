package com.example.myapplication

import android.annotation.SuppressLint
import android.content.ClipData.Item
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.util.Log
import android.view.View
import android.widget.Button
import android.widget.Toast
import androidx.drawerlayout.widget.DrawerLayout
import androidx.fragment.app.Fragment
import androidx.fragment.app.FragmentManager
import com.google.android.material.navigation.NavigationView
import com.google.android.material.snackbar.Snackbar


class MainActivity : AppCompatActivity() {
    @SuppressLint("MissingInflatedId")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
        val vueNav : NavigationView = findViewById(R.id.vue )

        val snack : Snackbar = Snackbar.make(findViewById(R.id.vueSnackBar), "Hello World",
            Snackbar.LENGTH_LONG)

        val fragHome = Home()
        val frag1 = Fragment1()
        val frag2 = Fragment2()

        remplaceFragment(fragHome)

        vueNav.setNavigationItemSelectedListener {

            findViewById<DrawerLayout>(R.id.drawer_layout).closeDrawer(vueNav)
            when(it.itemId) {

                R.id.item1 -> { remplaceFragment(fragHome)
                    Toast.makeText(applicationContext, "Home",
                    Toast.LENGTH_SHORT).show()}

                R.id.item2 ->{remplaceFragment(frag1)
                    Toast.makeText(applicationContext, "Page Video",
                        Toast.LENGTH_SHORT).show()}

                R.id.item3 ->{remplaceFragment(frag2)
                    Toast.makeText(applicationContext, "Page Map",
                        Toast.LENGTH_SHORT).show()

                }
            }
            true
        }
    }
    private fun remplaceFragment(fragment : Fragment)
    {
        val fragMan : FragmentManager = supportFragmentManager
        val fragTran = fragMan.beginTransaction()
        fragTran.replace(R.id.Fragment, fragment)
        fragTran.commit()
    }

}


