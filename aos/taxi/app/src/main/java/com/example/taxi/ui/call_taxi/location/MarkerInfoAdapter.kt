package com.example.taxi.ui.call_taxi.location

import com.example.taxi.R
import android.content.Context
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.ImageView
import android.widget.TextView
import com.naver.maps.map.overlay.InfoWindow


class MarkerInfoAdapter(context: Context,val mParent: ViewGroup, val distance: String, val cash: String) : InfoWindow.DefaultViewAdapter(context) {
    override fun getContentView(p0: InfoWindow): View {
        val view =
            LayoutInflater.from(context).inflate(R.layout.item_taxi_info, mParent, false) as View

        val tvDistance = view.findViewById<View>(R.id.tv_distance) as TextView
        val tvCash = view.findViewById<View>(R.id.tc_cash) as TextView

        tvDistance.text = distance
        tvCash.text = cash

        return view
    }
}