package com.example.taxi.ui.call_taxi.location

import androidx.navigation.fragment.findNavController
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.databinding.FragmentLocationTrackingTaxiBinding
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class LocationTrackingTaxiFragment : BaseFragment<FragmentLocationTrackingTaxiBinding>(R.layout.fragment_location_tracking_taxi) {
    override fun init() {
        findNavController().navigate(R.id.action_locationTrackingTaxiFragment_to_startDrivingTaxiFragment)
    }

}